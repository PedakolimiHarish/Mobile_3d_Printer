// start of firmware/src/main.cpp

#include <chrono>
#include <thread>
#include <iostream>
#include <optional>
#include <cstring>

// Core firmware headers
#include "firmware/machine_state.hpp"
#include "firmware/state_machine.hpp"
#include "firmware/command.hpp"
#include "firmware/command_executor.hpp"
#include "firmware/command_channel.hpp"
#include "firmware/execution_context.hpp"
#include "firmware/execution_loop.hpp"
#include "firmware/execution_result.hpp"
#include "firmware/execution_orchestrator.hpp"
#include "firmware/job.hpp"
#include "firmware/job_store.hpp"
#include "firmware/motion/ros_sim_motion_controller.hpp"
#include "firmware/extruder/extruder_controller_stub.hpp"
#include "firmware/safety/safety_state.hpp"
#include "firmware/safety/safety_monitor.hpp"
#include "firmware/job_ingress.hpp"
#include "firmware/job_ingress_state.hpp"
#include "firmware/job_cleanup.hpp"
#include "firmware/pause_snapshot.hpp"
#include "firmware/execution_gate.hpp"
#include "firmware/invariants.hpp"
#include "firmware/anchor_state.hpp"
#include "firmware/anchor_manager.hpp"
#include "firmware/segment_runner.hpp"
#include "firmware/execution_trace.hpp"
#include "firmware/persistence/persistent_execution_snapshot.hpp"

// IPC
#include "ipc/machine_state_ipc.h"
#include "ipc/state_publisher.hpp"
// #include "wbp_interfaces/ipc/job_ingress_ipc.h"
#include "ipc/job_ingress_ipc.hpp"

// Persistence
#include "firmware/persistence/persistent_state_io.hpp"

#include <signal.h>
#include <atomic>

std::atomic<bool> g_running(true);

void signal_handler(int)
{
    g_running.store(false);
}

using namespace firmware;
using namespace std::chrono_literals;

constexpr auto EXECUTION_TICK = std::chrono::milliseconds(50);

StateMachine *g_state_machine = nullptr;

StateMachine &get_state_machine()
{
    assert(g_state_machine);
    return *g_state_machine;
}

namespace firmware
{

    // --------- GLOBAL FIRMWARE OWNERSHIP ---------

    // Job g_active_job;
    MachineState g_machine_state;
    JobStore g_job_store;
    Job g_active_job;
    ExecutionContext g_execution_context;
    AnchorManager g_anchor_manager;

    MachineState &get_machine_state() { return g_machine_state; }
    JobStore &get_job_store() { return g_job_store; }
    JobIngressState &get_job_ingress_state();
    ExecutionContext &get_execution_context() { return g_execution_context; }
    AnchorManager &get_anchor_manager() { return g_anchor_manager; }
} // namespace firmware

static uint32_t last_job_seq = 0;

static void poll_job_ingress()
{
    static bool first_read = true;

    if (first_read)
    {
        first_read = false;
        return;
    }

    /* if (ipc->size == 0)
        return;
    auto *ipc = firmware::ipc::get_job_ipc(); */

    auto *ipc = firmware::ipc::get_job_ipc();

    if (!ipc || ipc->magic != WBP_JOB_IPC_MAGIC)
        return;

    if (ipc->size == 0)
        return;

    if (!ipc || ipc->magic != WBP_JOB_IPC_MAGIC)
        return;

    /* if (ipc->ingress_complete != 1)
        return; */

    if (ipc->sequence == last_job_seq)
        return;

    std::cerr << "[FW] Job IPC seq=" << ipc->sequence
              << " size=" << ipc->size << "\n";

    /* last_job_seq = ipc->sequence; */

    if (ipc->size == 0 || ipc->size > WBP_JOB_MAX_SIZE)
        return;

    std::vector<uint8_t> blob(
        ipc->job_blob,
        ipc->job_blob + ipc->size);

    auto result = firmware::submit_job(blob, ipc->source);

    if (result.accepted)
    {
        std::cerr << "[FW] Job accepted id=" << result.job_id << std::endl;
        last_job_seq = ipc->sequence;
    }
    else
    {
        std::cerr << "[FW] Job rejected reason=" << result.rejection_reason << std::endl;
    }

    // 🔥 CRITICAL: clear flag
    // ipc->ingress_complete = 0;
}

static void classify_boot_state(
    MachineState &ms,
    ExecutionContext &exec)
{
    exec.reset_soft();
    ms.recovery_in_progress = false;

    ms.resume_available = false;
    // 🔒 Phase 29.C — boot NEVER exits RECOVERING automatically
    ms.resume_consumed = false; // 🔒 REQUIRED (Phase 25.c / 25.e)
    ms.system = SystemState::READY;

    ms.operator_recovery_required = false;
    ms.resume_permanently_disabled = false;
    ms.operator_actions = {};

    PersistentExecutionSnapshot snap{};
    if (!load_execution_snapshot(snap))
        return;

    if (snap.magic != SNAPSHOT_MAGIC ||
        snap.system_at_snapshot != SystemState::PAUSED)
    {
        clear_execution_snapshot();
        return;
    }

    // 🔒 Phase 26.C — snapshot sanity (AUTHORITATIVE)
    if (snap.job_id != ms.job_id ||
        snap.job_schema_version != ms.job_schema_version ||
        std::memcmp(snap.job_hash,
                    ms.job_hash,
                    sizeof(ms.job_hash)) != 0)
    {
        clear_execution_snapshot();
        return;
    }

    ms.system = SystemState::PAUSED;
    ms.resume_available = true;

    std::cerr << "[FW] Resume available at cursor="
              << snap.cursor << "\n";
}

int main()
{
    std::cerr << "[FW] Starting firmware\n";

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    auto &exec_ctx = firmware::get_execution_context();

    /* ---------------- IPC INIT ---------------- */

    auto *ipc = firmware::ipc::init_ipc_state();
    if (!ipc)
    {
        std::cerr << "[FW] IPC state init failed\n";
        return 1;
    }

    if (!firmware::ipc::init_ipc_command())
    {
        std::cerr << "[FW] IPC command init failed\n";
        return 1;
    }

    g_machine_state.boot_id++;

    // 🔒 PHASE 22.D — boot classification
    classify_boot_state(g_machine_state, exec_ctx);

    std::optional<PauseSnapshot> pause_snapshot;

    if (g_machine_state.system == SystemState::PAUSED)
    {
        PersistentExecutionSnapshot snap{};
        bool ok = load_execution_snapshot(snap);
        assert(ok);

        pause_snapshot.emplace();
        pause_snapshot->job_id = snap.job_id;
        pause_snapshot->layer = snap.layer;
        pause_snapshot->segment = snap.segment;
        pause_snapshot->intra_progress = snap.unit_progress;
        pause_snapshot->volume_checksum = snap.checksum;

        std::cerr << "[FW] PauseSnapshot reconstructed\n";
    }

    if (!firmware::ipc::init_ipc_job_ingress())
    {
        std::cerr << "[FW] Job IPC init failed\n";
        return 1;
    }
    std::cerr << "[FW] Job IPC ready\n"; // TEMP diagnostic

    /* auto *job_ipc = firmware::ipc::get_job_ipc();

    if (!job_ipc || job_ipc->magic != WBP_JOB_IPC_MAGIC)
    {
        std::memset(job_ipc, 0, sizeof(*job_ipc));
        job_ipc->magic = WBP_JOB_IPC_MAGIC;
    } */

    auto *job_ipc = firmware::ipc::get_job_ipc();

    if (!job_ipc)
    {
        std::cerr << "[FW] Job IPC null\n";
        return 1;
    }

    // 🔥 ALWAYS reset ingress region on boot
    std::memset(job_ipc, 0, sizeof(*job_ipc));
    job_ipc->magic = WBP_JOB_IPC_MAGIC;

    std::cerr << "[FW] Job IPC reset on boot\n";

    /* ---------------- CORE OBJECTS ---------------- */

    PrintedVolume printed_volume(1000, 1000, 0.01);
    // ExecutionContext exec_ctx;

    // StateMachine sm(g_machine_state);

    StateMachine sm(g_machine_state, exec_ctx);
    g_state_machine = &sm;

    CommandExecutor executor(
        g_machine_state,
        printed_volume,
        exec_ctx);

    CommandChannel command_channel(
        sm,
        executor,
        printed_volume);

    /* ---------------- EXECUTION PLANE ---------------- */

    RosSimMotionController motion;
    ExtruderControllerStub extruder;

    SegmentRunner segment_runner(exec_ctx, g_machine_state);

    ExecutionOrchestrator orchestrator(
        exec_ctx,
        sm,
        g_machine_state,
        pause_snapshot,
        motion);

    // 🔒 Phase 23.C — ownership transfer
    exec_ctx.owned_by_executor = false; // owned by orchestrator only after RUNNING

    std::cerr << "[FW] Firmware idle, waiting for commands\n";

    SafetyInputs raw_safety;

    auto next_tick = std::chrono::steady_clock::now();

    // DEBUG TEST — remove after validation
    // exec_ctx.cursor = 42; // should ASSERT immediately

    while (g_running.load())
    {
        next_tick += EXECUTION_TICK;

        // 1️⃣ Ingress
        command_channel.poll();

        static bool first_cycle = true;

        poll_job_ingress();

        // 2️⃣ Execution gate (arms once per job)
        activate_job_execution(
            firmware::get_job_ingress_state(),
            firmware::get_job_store(),
            g_machine_state,
            exec_ctx);

        // 3️⃣ Safety
        SafetyInputs safety = evaluate_safety(raw_safety);
        if (safety.status == SafetyStatus::FAULT)
        {
            sm.request_fault(0xF001);
        }

        // 4️⃣ Orchestrator FIRST
        ExecutionResult exec_result = ExecutionResult::BLOCKED;

        /* std::cerr << "[DBG] owned=" << exec_ctx.owned_by_executor
                  << " phase=" << int(exec_ctx.phase)
                  << " sys=" << int(g_machine_state.system) << "\n"; */

        // 5️⃣ Arm → RUNNING transition
        if (g_machine_state.system == SystemState::EXECUTING_PRINT &&
            exec_ctx.phase == ExecutionPhase::IDLE &&
            exec_ctx.armed &&
            exec_ctx.cursor < g_active_job.total_segments)
        {
            exec_ctx.phase = ExecutionPhase::RUNNING;
            exec_ctx.armed = false;
            exec_ctx.owned_by_executor = true;
            std::cerr << "[EXEC] Execution entered RUNNING\n";
        }

        // 6️⃣ Execute ONE step
        if (safety.status == SafetyStatus::SAFE &&
            g_machine_state.system == SystemState::EXECUTING_PRINT &&
            exec_ctx.phase == ExecutionPhase::RUNNING &&
            !exec_ctx.abort_in_progress)
        {
            exec_result =
                execution_step(exec_ctx, g_active_job, motion, extruder);

            trace_execution(exec_ctx, g_machine_state, exec_result);
        }

        motion.poll();

        // 🔒 UPDATE MOTION IDLE STATE (CRITICAL)
        g_machine_state.motion_idle = motion.is_ready();

        // 7️⃣ Orchestrator consumes result
        orchestrator.step(exec_result);

        g_machine_state.motion_idle = motion.is_ready();

        if (g_machine_state.system == SystemState::IDLE &&
            g_machine_state.job.state == JobState::COMPLETED)
        {
            assert(exec_ctx.phase == ExecutionPhase::IDLE);
            assert(exec_ctx.cursor == 0);
            assert(exec_ctx.owned_by_executor == false);
        }

        enforce_execution_ownership(g_machine_state, exec_ctx);

        // 8️⃣ Invariants LAST — but NEVER block arming
        if (g_machine_state.system == SystemState::EXECUTING_PRINT &&
            exec_ctx.phase == ExecutionPhase::RUNNING)
        {
            enforce_invariants(g_machine_state, exec_ctx);
        }

        // 9️⃣ Pause guard cleanup
        sm.clear_pause_pending_if_stable(exec_ctx);

        g_machine_state.resume_permanently_disabled =
            g_machine_state.resume_consumed;

        sm.update_operator_actions();

        // 🔟 Publish state
        ipc::publish_ipc_state(g_machine_state);

        std::this_thread::sleep_until(next_tick);
    }
}

// end of firmware/src/main.cpp