// firmware/include/firmware/state_machine.hpp
#pragma once
#include "firmware/machine_state.hpp"
#include "firmware/printed_volume.hpp"
#include "firmware/pause_snapshot.hpp"
#include "firmware/job.hpp"
namespace firmware
{
    struct ExecutionContext; // forward declaration

    enum class TransitionResult : uint8_t
    {
        ACCEPTED,
        REJECTED,
        FAULTED
    };

    // StateMachine is a lifecycle authority ONLY.
    // It MUST NOT mutate ExecutionContext state.
    // All execution mutation is owned by executor + orchestrator.

    class StateMachine
    {
    public:
        // Primary constructor (no execution context)
        explicit StateMachine(MachineState &state);

        // Optional wiring constructor (Phase 22.A)
        StateMachine(MachineState &state, ExecutionContext &exec);

        TransitionResult request_start_job();
        TransitionResult request_pause();
        TransitionResult request_resume(const PauseSnapshot &snapshot, const PrintedVolume &volume);
        TransitionResult request_abort();
        TransitionResult request_fault(uint32_t fault_code);
        TransitionResult request_job_complete();
        TransitionResult request_localization_mode(LocalizationMode mode);
        TransitionResult request_set_localization_mode(LocalizationMode mode);
        TransitionResult request_clear_job();
        TransitionResult request_recover();
        TransitionResult request_recovery_complete();
        TransitionResult accept_new_job(const Job &job);

        void finalize_abort();
        void reset_execution(ExecutionContext &ctx);
        void record_command_outcome(uint32_t seq, CommandOutcome outcome);
        void clear_pause_pending_if_stable(const ExecutionContext &ctx);
        void update_operator_actions();

        const MachineState &state() const;
        bool is_pause_pending() const { return pause_pending_; }

    private:
        MachineState &state_;
        ExecutionContext *exec_ctx_{nullptr};

        bool pause_pending_{false};
        bool can_start_job() const;
        bool can_pause() const;
        bool can_resume(const PauseSnapshot &snapshot,
                        const PrintedVolume &volume) const;
    };

} // namespace firmware