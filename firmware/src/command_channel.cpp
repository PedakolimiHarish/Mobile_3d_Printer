// start of file: firmware/src/command_channel.cpp
#include "firmware/command_channel.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

#include "firmware/state_machine.hpp"
#include "firmware/command_executor.hpp"
#include "firmware/printed_volume.hpp"

namespace firmware
{

    CommandChannel::CommandChannel(StateMachine &sm,
                                   CommandExecutor &executor,
                                   const PrintedVolume &printed_volume)
        : state_machine_(sm), executor_(executor), printed_volume_(printed_volume)
    {
        shm_fd_ = shm_open(WBP_COMMAND_SHM_NAME, O_RDONLY, 0);
        if (shm_fd_ < 0)
            return;

        shm_ptr_ = static_cast<wbp_command_ipc_t *>(
            mmap(nullptr,
                 sizeof(wbp_command_ipc_t),
                 PROT_READ,
                 MAP_SHARED,
                 shm_fd_,
                 0));
    }

    void CommandChannel::poll()
    {
        if (!shm_ptr_)
            return;

        const uint32_t seq = shm_ptr_->seq;
        if (seq <= last_seen_seq_)
            return;

        wbp_command_ipc_t local;
        std::memcpy(&local, shm_ptr_, sizeof(local));

        if (local.seq != seq)
            return;

        last_seen_seq_ = seq;

        if (!validate(local))
            return;

        handle_command(local);
    }

    bool CommandChannel::validate(const wbp_command_ipc_t &cmd) const
    {
        if (cmd.abi_version != WBP_COMMAND_IPC_ABI_VERSION)
            return false;

        if (cmd.flags != 0)
            return false;

        return true;
    }
    static bool command_allowed_in_state(
        SystemState sys,
        uint32_t cmd)
    {
        switch (sys)
        {
        case SystemState::FAULT:
            return cmd == WBP_CMD_RECOVER ||
                   cmd == WBP_CMD_CLEAR_JOB;

        case SystemState::RECOVERING:
            return cmd == WBP_CMD_CLEAR_JOB;

        case SystemState::READY:
            return cmd == WBP_CMD_START ||
                   cmd == WBP_CMD_CLEAR_JOB ||
                   cmd == WBP_CMD_SET_LOCALIZATION_MODE;

        case SystemState::EXECUTING_PRINT:
            return cmd == WBP_CMD_PAUSE ||
                   cmd == WBP_CMD_ABORT;

        case SystemState::PAUSED:
            return cmd == WBP_CMD_RESUME ||
                   cmd == WBP_CMD_ABORT;

        default:
            return false;
        }
    }

    void CommandChannel::handle_command(const wbp_command_ipc_t &cmd)
    {
        const auto &ms = state_machine_.state();

        if (!command_allowed_in_state(ms.system, cmd.command))
        {
            state_machine_.record_command_outcome(
                cmd.seq, CommandOutcome::REJECTED);
            return;
        }

        const auto &oa = ms.operator_actions;
        switch (cmd.command)
        {
        case WBP_CMD_START:
        {
            if (!oa.can_start)
            {
                state_machine_.record_command_outcome(
                    cmd.seq, CommandOutcome::REJECTED);
                return;
            }
            auto r = state_machine_.request_start_job();
            state_machine_.record_command_outcome(
                cmd.seq,
                r == TransitionResult::ACCEPTED
                    ? CommandOutcome::ACCEPTED
                    : CommandOutcome::REJECTED);
            break;
        }

        case WBP_CMD_PAUSE:
        {
            auto r = state_machine_.request_pause();
            state_machine_.record_command_outcome(
                cmd.seq,
                r == TransitionResult::ACCEPTED
                    ? CommandOutcome::ACCEPTED
                    : CommandOutcome::REJECTED);
            break;
        }

        case WBP_CMD_RESUME:
        {
            if (!oa.can_resume)
            {
                state_machine_.record_command_outcome(
                    cmd.seq, CommandOutcome::REJECTED);
                return;
            }
            auto r = state_machine_.request_resume(
                PauseSnapshot{}, printed_volume_);

            state_machine_.record_command_outcome(
                cmd.seq,
                r == TransitionResult::ACCEPTED
                    ? CommandOutcome::ACCEPTED
                    : CommandOutcome::REJECTED);
            break;
        }

        case WBP_CMD_ABORT:
        {
            auto r = state_machine_.request_abort();

            state_machine_.record_command_outcome(
                cmd.seq,
                r == TransitionResult::ACCEPTED
                    ? CommandOutcome::ACCEPTED
                    : CommandOutcome::REJECTED);
            break;
        }

        case WBP_CMD_CLEAR_JOB:
        {
            if (!oa.can_clear_job)
            {
                state_machine_.record_command_outcome(
                    cmd.seq, CommandOutcome::REJECTED);
                return;
            }
            auto r = state_machine_.request_clear_job();

            state_machine_.record_command_outcome(
                cmd.seq,
                r == TransitionResult::ACCEPTED
                    ? CommandOutcome::ACCEPTED
                    : CommandOutcome::REJECTED);
            break;
        }

        case WBP_CMD_SET_LOCALIZATION_MODE:
        {
            auto mode =
                static_cast<LocalizationMode>(cmd.localization_mode);

            auto r =
                state_machine_.request_set_localization_mode(mode);

            state_machine_.record_command_outcome(
                cmd.seq,
                r == TransitionResult::ACCEPTED
                    ? CommandOutcome::ACCEPTED
                    : CommandOutcome::REJECTED);

            break;
        }

        case WBP_CMD_RECOVER:
        {
            if (!oa.can_recover)
            {
                state_machine_.record_command_outcome(
                    cmd.seq, CommandOutcome::REJECTED);
                return;
            }

            auto r = state_machine_.request_recover();

            state_machine_.record_command_outcome(
                cmd.seq,
                r == TransitionResult::ACCEPTED
                    ? CommandOutcome::ACCEPTED
                    : CommandOutcome::REJECTED);
            break;
        }

        default:
        {
            state_machine_.record_command_outcome(
                cmd.seq, CommandOutcome::REJECTED);
            break;
        }
        }
    }

}

// end of firmware/src/command_channel.cpp