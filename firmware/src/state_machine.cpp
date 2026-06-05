// start of firmware/src/state_machine.cpp
#include "firmware/state_machine.hpp"
#include "firmware/execution_context.hpp"
#include "firmware/job_ingress_state.hpp"
#include "firmware/job_store.hpp"
#include "firmware/job.hpp"
#include "firmware/persistence/persistent_execution_snapshot.hpp"
#include <iostream>
#include <cstring>
#include <cassert>

namespace firmware
{

    // Constructor
    StateMachine::StateMachine(MachineState &state)
        : state_(state)
    {
    }
    StateMachine::StateMachine(MachineState &state, ExecutionContext &exec)
        : state_(state), exec_ctx_(&exec)
    {
    }

    // -------------------- START JOB --------------------
    // Phase 22.B invariant:
    // ExecutionContext reset is ONLY permitted during START_JOB.
    // No other command may reset cursor, phase, or progress.

    TransitionResult StateMachine::request_start_job()
    {
        extern Job g_active_job;

        if (!state_.job_present)
        {
            std::cerr << "[SM] START rejected — no job\n";
            return TransitionResult::REJECTED;
        }

        if (exec_ctx_ && exec_ctx_->cursor >= g_active_job.total_segments)
        {
            std::cerr << "[SM] START blocked — job already completed\n";
            return TransitionResult::REJECTED;
        }

        if (!state_.job_present)
        {
            std::cerr << "[SM] START rejected: no job present\n";
            return TransitionResult::REJECTED;
        }

        if (state_.job.state == JobState::COMPLETED)
        {
            std::cerr << "[SM] START rejected: job already completed\n";
            return TransitionResult::REJECTED;
        }

        if (!can_start_job())
        {
            std::cerr << "[SM] START rejected\n";
            return TransitionResult::REJECTED;
        }

        // 🔒 Phase 26 — starting a job kills resume forever
        state_.resume_available = false;
        state_.resume_consumed = false;

        state_.system = SystemState::EXECUTING_PRINT;
        state_.job.state = JobState::ACTIVE;

        // Minimal execution marker for UI + observers
        if (state_.job.layer == 0 && state_.job.segment == 0)
        {
            state_.job.layer = 1;
        }

        std::cerr << "[SM] START accepted, system="
                  << static_cast<int>(state_.system)
                  << " safety=" << static_cast<int>(state_.safety.state)
                  << "\n";

        return TransitionResult::ACCEPTED;
    }

    bool StateMachine::can_start_job() const
    {
        if (state_.system != SystemState::READY)
            return false;

        if (state_.operator_recovery_required)
            return false;

        return true;
    }

    TransitionResult StateMachine::request_localization_mode(LocalizationMode mode)
    {
        // Cannot change mode during execution
        if (state_.system == SystemState::EXECUTING_PRINT ||
            state_.system == SystemState::RESUMING)
        {
            return TransitionResult::REJECTED;
        }

        // ANCHORED requires anchors to be valid (Phase 18.D)
        if (mode == LocalizationMode::ANCHORED)
        {
            if (!state_.anchors_valid) // <-- comes from 18.D
                return TransitionResult::REJECTED;
        }

        state_.localization_mode = mode;
        return TransitionResult::ACCEPTED;
    }

    // -------------------- PAUSE --------------------
    TransitionResult StateMachine::request_pause()
    {
        if (state_.system == SystemState::RECOVERING)
            return TransitionResult::REJECTED;

        if (!can_pause())
            return TransitionResult::REJECTED;
        state_.system = SystemState::PAUSED;
        state_.resume_available = true;
        std::cerr << "[SM] PAUSE accepted\n";
        return TransitionResult::ACCEPTED;
    }

    bool StateMachine::can_pause() const
    {
        return state_.system == SystemState::EXECUTING_PRINT;
    }

    // -------------------- RESUME --------------------

    TransitionResult StateMachine::request_resume(
        const PauseSnapshot &snapshot,
        const PrintedVolume &volume)
    {
        if (state_.system == SystemState::RECOVERING)
            return TransitionResult::REJECTED;

        if (!can_resume(snapshot, volume))
        {
            std::cerr << "[SM] RESUME rejected\n";
            return TransitionResult::REJECTED;
        }

        state_.system = SystemState::RESUMING; // 🔥 REQUIRED
        state_.resume_available = false;
        state_.resume_consumed = true;
        std::cerr << "[SM] RESUME accepted\n";
        return TransitionResult::ACCEPTED;
    }

    TransitionResult StateMachine::request_clear_job()
    {
        if (state_.system != SystemState::READY &&
            state_.system != SystemState::IDLE &&
            state_.system != SystemState::RECOVERING &&
            state_.system != SystemState::FAULT)
            return TransitionResult::REJECTED;

        // Kill execution context
        if (exec_ctx_)
        {
            exec_ctx_->reset_soft();
        }

        clear_execution_snapshot();

        // 🔒 HARD RESET JOB
        state_.job = {};
        state_.job_present = false;

        // Clear job store (authoritative payload)
        get_job_store().clear();

        // Clear ingress latch
        auto &js = get_job_ingress_state();
        js.job_loaded = false;
        js.job_id = 0;
        js.schema_version = 0;
        js.last_rejection[0] = '\0';
        // Clear recovery flags
        state_.recovery_in_progress = false;
        state_.operator_recovery_required = false;

        // Resume is permanently dead after fault
        state_.resume_available = false;
        state_.resume_consumed = true;

        state_.system = SystemState::READY;

        std::cerr << "[SM] CLEAR_JOB completed\n";
        return TransitionResult::ACCEPTED;
    }

    TransitionResult StateMachine::request_recover()
    {
        if (state_.system != SystemState::FAULT)
            return TransitionResult::REJECTED;

        // 🔒 Kill execution state immediately
        if (exec_ctx_)
        {
            exec_ctx_->reset_soft();
        }

        clear_execution_snapshot();

        state_.system = SystemState::RECOVERING;
        state_.recovery_in_progress = true;

        // 🔒 Resume is permanently dead after fault
        state_.resume_available = false;
        state_.resume_consumed = true;

        state_.operator_recovery_required = true;

        std::cerr << "[SM] RECOVER accepted\n";
        return TransitionResult::ACCEPTED;
    }

    TransitionResult StateMachine::request_recovery_complete()
    {
        if (state_.system != SystemState::RECOVERING)
            return TransitionResult::REJECTED;

        // 🔒 Enforce job cleanup
        if (state_.job_present)
        {
            std::cerr << "[SM] RECOVERY blocked: job still present\n";
            return TransitionResult::REJECTED;
        }

        if (exec_ctx_)
            exec_ctx_->reset_soft();

        state_.recovery_in_progress = false;
        state_.operator_recovery_required = false;

        state_.system = SystemState::READY;

        std::cerr << "[SM] RECOVERY completed, system READY\n";
        return TransitionResult::ACCEPTED;
    }

    bool StateMachine::can_resume(
        const PauseSnapshot &snap,
        const PrintedVolume &) const
    {
        // 🔒 Phase 28.B — FAULT blocks resume
        if (state_.system == SystemState::FAULT)
            return false;

        if (state_.system == SystemState::RECOVERING)
            return false;

        if (state_.system != SystemState::PAUSED)
            return false;

        if (!state_.resume_available)
            return false;

        if (state_.resume_consumed)
            return false;

        // Job ID must still match
        if (state_.job.job_id != snap.job_id)
            return false;

        // Safety: executor must be idle
        if (exec_ctx_ && exec_ctx_->owned_by_executor)
            return false;

        return true;
    }

    void StateMachine::clear_pause_pending_if_stable(const ExecutionContext &ctx)
    {
        if (pause_pending_ &&
            state_.system == SystemState::PAUSED &&
            ctx.phase == ExecutionPhase::PAUSED)
        {
            pause_pending_ = false;
        }
    }

    void StateMachine::record_command_outcome(
        uint32_t seq,
        CommandOutcome outcome)
    {
        state_.last_command_seq = seq;
        state_.last_command_outcome = outcome;
    }

    void StateMachine::finalize_abort()
    {
        // 🔒 PHASE 25.e — abort kills resume
        state_.resume_available = false;
        state_.resume_consumed = false;
        state_.system = SystemState::IDLE;
        state_.job.state = JobState::NONE;
    }

    TransitionResult StateMachine::request_set_localization_mode(
        LocalizationMode mode)
    {
        // Cannot change mode during execution
        if (state_.system == SystemState::EXECUTING_PRINT ||
            state_.system == SystemState::RESUMING)
        {
            return TransitionResult::REJECTED;
        }

        // Anchored requires valid anchors
        if (mode == LocalizationMode::ANCHORED &&
            !state_.anchors_valid)
        {
            return TransitionResult::REJECTED;
        }

        // Idempotent accept
        if (state_.localization_mode == mode)
        {
            return TransitionResult::ACCEPTED;
        }

        state_.localization_mode = mode;
        return TransitionResult::ACCEPTED;
    }

    // -------------------- ABORT --------------------

    TransitionResult StateMachine::request_abort()
    {
        if (state_.system == SystemState::IDLE)
            return TransitionResult::REJECTED;

        if (state_.system == SystemState::RECOVERING)
            return TransitionResult::REJECTED;

        if (state_.system != SystemState::EXECUTING_PRINT &&
            state_.system != SystemState::PAUSED)
            return TransitionResult::REJECTED;

        state_.system = SystemState::ABORTING;

        // ExecutionContext teardown happens elsewhere
        return TransitionResult::ACCEPTED;
    }

    // -------------------- FAULT --------------------

    TransitionResult StateMachine::request_fault(uint32_t fault_code)
    {
        state_.system = SystemState::FAULT;
        state_.safety.state = SafetyState::FAULT;
        state_.safety.fault_code = fault_code;

        // 🔒 Phase 28.A — fault invalidates resume forever
        state_.resume_available = false;
        state_.resume_consumed = true;

        state_.last_command_outcome = CommandOutcome::FAULTED;

        state_.operator_recovery_required = true;
        state_.resume_permanently_disabled = true;

        return TransitionResult::ACCEPTED;
    }

    TransitionResult StateMachine::request_job_complete()
    {
        // Job completion is only valid during execution
        if (state_.system != SystemState::EXECUTING_PRINT)
        {
            return TransitionResult::REJECTED;
        }

        // Finalize job
        state_.job.state = JobState::COMPLETED;

        // 🔒 PHASE 25.e — completion closes resume lifecycle
        state_.resume_available = false;
        state_.resume_consumed = false;

        // Return system to ready
        state_.system = SystemState::READY;
        std::cerr << "[SM] Job completed, system READY\n";
        return TransitionResult::ACCEPTED;
    }

    const MachineState &StateMachine::state() const
    {
        return state_;
    }

    static OperatorActions derive_operator_actions(const MachineState &ms)
    {
        OperatorActions a{};

        if (ms.job_present && ms.job.state == JobState::COMPLETED)
        {
            a.can_start = false;    // ✅ cannot restart completed job
            a.can_clear_job = true; // ✅ must clear first
        }

        if (ms.system == SystemState::FAULT)
        {
            a.can_submit_job = false;
            a.can_start = false;
            a.can_resume = false;
            a.can_clear_job = true;
            a.can_recover = true;
            return a;
        }

        if (ms.system == SystemState::RECOVERING)
        {
            a.can_submit_job = false;
            a.can_start = false;
            a.can_resume = false;
            a.can_clear_job = true;
            a.can_recover = false;
            return a;
        }

        a.can_submit_job = true;
        a.can_start = true;
        a.can_clear_job = true;
        a.can_recover = false;
        a.can_resume = !ms.resume_consumed;

        return a;
    }

    void StateMachine::update_operator_actions()
    {
        state_.operator_actions = derive_operator_actions(state_);
    }

} // namespace firmware

// end of firmware/src/state_machine.cpp