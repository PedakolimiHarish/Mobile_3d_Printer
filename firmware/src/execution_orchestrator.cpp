// start of firmware/src/execution_orchestrator.cpp

// Phase 24.A invariant:
// ExecutionOrchestrator is the ONLY component allowed to
// emit job completion and finalize execution context.

#include "firmware/execution_orchestrator.hpp"
#include "firmware/machine_state.hpp"
#include "firmware/pause_snapshot.hpp"
#include "firmware/anchor_state.hpp"
#include "firmware/printed_volume.hpp"
#include "firmware/persistence/persistent_execution_snapshot.hpp"
#include "firmware/motion/motion_controller.hpp"
#include "firmware/job_store.hpp"
#include "ipc/job_ingress_ipc.hpp"
#include "firmware/job_cleanup.hpp"

#include <cstring>
#include <iostream>
#include <cassert>
namespace firmware
{

    ExecutionOrchestrator::ExecutionOrchestrator(
        ExecutionContext &exec_ctx,
        StateMachine &sm,
        MachineState &ms,
        std::optional<PauseSnapshot> &pause_snapshot,
        MotionController &motion)
        : ctx_(exec_ctx),
          sm_(sm),
          machine_state_(ms),
          pause_snapshot_(pause_snapshot),
          motion_(motion)
    {
    }

    void ExecutionOrchestrator::step(ExecutionResult last_result)
    { // Orchestrator may run in pre-execution states (RESUMING / PAUSED)

        // std::cerr << "[ORCH DBG] system=" << int(machine_state_.system) << "\n";

        auto &ctx = ctx_;
        extern Job g_active_job;

        assert_execution_ownership_valid(ctx_);

        // ---------- FAULT / RECOVERING ----------
        if (machine_state_.system == SystemState::FAULT ||
            machine_state_.system == SystemState::RECOVERING)
        {
            return; // 🔒 Phase 29.b — hard stop, no execution
        }

        // =========================================================
        // START EXECUTION (HARD GATE — MUST BE EARLY)
        // =========================================================
        if (machine_state_.system == SystemState::EXECUTING_PRINT)
        {
            // 🔥 HARD GUARD — prevent restart after completion
            if (machine_state_.job.state == JobState::COMPLETED)
                return;

            if (!ctx_.owned_by_executor &&
                ctx_.phase == ExecutionPhase::IDLE &&
                ctx_.cursor < g_active_job.total_segments &&
                (ctx_.armed || ctx_.cursor == 0)) // 🔥 FIX
            {
                std::cerr << "[ORCH] START/RESUME → execution armed\n";

                ctx_.phase = ExecutionPhase::RUNNING;
                ctx_.owned_by_executor = true;
                ctx_.armed = false; // 🔥 consume arm
            }
        }

        // ---------- ABORT ----------
        if (machine_state_.system == SystemState::ABORTING)
        {
            // 🔒 Tracks whether we already issued abort
            static bool abort_started_once = false;

            // ---------- START ----------
            if (!abort_started_once)
            {
                std::cerr << "[ORCH] ABORT start\n";

                ctx.abort_in_progress = true;

                ctx.phase = ExecutionPhase::IDLE;
                ctx.owned_by_executor = false;

                pause_snapshot_.reset();
                clear_execution_snapshot();

                motion_.abort();
                motion_.set_abort_active(true);

                abort_started_once = true;
            }

            // ---------- DRAIN DETECTION ----------
            if (ctx.abort_in_progress && machine_state_.motion_idle)
            {
                std::cerr << "[ORCH] Abort drain detected\n";
                ctx.abort_in_progress = false;
            }

            // ---------- FINALIZE ----------
            if (!ctx.abort_in_progress && abort_started_once)
            {
                std::cerr << "[ORCH] ABORT finalized\n";

                motion_.set_abort_active(false);
                machine_state_.last_completion_reason = CompletionReason::ABORTED;

                sm_.finalize_abort();

                abort_started_once = false; // 🔁 reset for next cycle
            }

            return;
        }

        // =========================================================
        // PAUSED — AUTHORITATIVE HARD STOP (OPTION A)
        // =========================================================
        if (machine_state_.system == SystemState::PAUSED)
        {
            // 1. Capture snapshot ONCE
            if (!pause_snapshot_)
            {
                pause_snapshot_.emplace();
                pause_snapshot_->job_id = machine_state_.job.job_id;
                pause_snapshot_->segment = ctx.cursor;
                pause_snapshot_->intra_progress = ctx.unit_progress;

                PersistentExecutionSnapshot ps{};
                ps.magic = SNAPSHOT_MAGIC;
                ps.boot_id = machine_state_.boot_id;
                ps.job_id = machine_state_.job.job_id;
                ps.cursor = ctx.cursor;
                ps.unit_progress = ctx.unit_progress;
                ps.system_at_snapshot = SystemState::PAUSED;

                persist_execution_snapshot(ps);

                std::cerr << "[ORCH] Pause snapshot captured + persisted\n";
            }

            // 2. Revoke execution ownership EVERY tick
            if (ctx.owned_by_executor || ctx.armed)
            {
                ctx.phase = ExecutionPhase::IDLE;
                ctx.owned_by_executor = false;
                ctx.armed = false;

                std::cerr << "[ORCH] Execution paused, ownership released\n";
            }

            return; // 🔒 HARD STOP — NOTHING BELOW MAY RUN
        }

        // =========================================================
        // RESUME (single-shot)
        // =========================================================
        if (machine_state_.system == SystemState::RESUMING)
        {
            if (ctx.owned_by_executor)
                return;

            PersistentExecutionSnapshot snap{};
            if (!load_execution_snapshot(snap))
            {
                sm_.request_fault(0xE060);
                return;
            }

            ctx.reset_hard();
            ctx.cursor = snap.cursor;
            ctx.unit_progress = snap.unit_progress;

            ctx.phase = ExecutionPhase::IDLE;
            ctx.armed = true;
            ctx.owned_by_executor = false;
            ctx.completion_emitted = false;

            clear_execution_snapshot();
            pause_snapshot_.reset(); // 🔒 REQUIRED
            machine_state_.resume_available = false;
            // ✅ ADD THIS LINE HERE
            machine_state_.resume_consumed = true;
            machine_state_.system = SystemState::EXECUTING_PRINT;
            std::cerr << "[RESUME DBG] cursor=" << ctx.cursor << "\n";

            std::cerr << "[ORCH] Resume restored and armed\n";
            return;
        }

        // ---------- FAULT ----------
        if (machine_state_.system == SystemState::FAULT)
            return;

        // ---------- PROGRESS TRACKING ----------
        if (ctx.phase == ExecutionPhase::RUNNING)
        {
            if (last_result == ExecutionResult::PROGRESSED)
            {
                ctx.blocked_ticks = 0;
            }
            else if (last_result == ExecutionResult::BLOCKED)
            {
                ctx.blocked_ticks++;
            }
        }

        // ---------- EXECUTION RESULT ----------
        if (ctx.phase != ExecutionPhase::RUNNING)
        {

            return;
        }

        // ---------- MOTION-BASED COMPLETION ----------
        if (ctx.phase == ExecutionPhase::RUNNING &&
            ctx.cursor >= g_active_job.total_segments &&
            machine_state_.motion_idle)
        {
            std::cerr << "[ORCH] reason=NORMAL (all segments completed + drained)\n";

            machine_state_.last_completion_reason = CompletionReason::NORMAL;

            ctx.phase = ExecutionPhase::COMPLETED;

            sm_.request_job_complete();

            auto &store = get_job_store();
            free_job_segments(g_active_job);
            store.clear();

            machine_state_.job_present = false;
            machine_state_.job_id = 0;
            machine_state_.job_schema_version = 0;
            std::memset(machine_state_.job_hash, 0, sizeof(machine_state_.job_hash));

            machine_state_.job.layer = 0;
            machine_state_.job.segment = 0;
            machine_state_.job.state = JobState::NONE;

            std::cerr << "[ORCH] Job cleared (true completion)\n";

            ctx.reset_hard();
            ctx.owned_by_executor = false;
            ctx.cursor = 0; // 🔒 prevent restart loop
            ctx.phase = ExecutionPhase::IDLE;

            return;
        }

        // ---------- STARVED (explicit) ----------
        if (last_result == ExecutionResult::STARVED)
        {
            std::cerr << "[ORCH] reason=STARVATION (explicit)\n";

            machine_state_.last_completion_reason = CompletionReason::STARVATION;

            ctx.phase = ExecutionPhase::COMPLETED;

            sm_.request_job_complete();

            ctx.reset_hard();
            ctx.owned_by_executor = false;

            return;
        }
    }

} // namespace firmware

// end of firmware/src/execution_orchestrator.cpp