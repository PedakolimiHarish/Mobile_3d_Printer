// firmware/include/firmware/command_channel.hpp
#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "firmware/command.hpp"
#include "firmware/pause_snapshot.hpp"
#include "ipc/command_ipc.h"

namespace firmware
{

    class StateMachine;
    class CommandExecutor;
    class PrintedVolume;

    class CommandChannel
    {
    public:
        CommandChannel(StateMachine &sm,
                       CommandExecutor &executor,
                       const PrintedVolume &printed_volume);

        void poll(); // ✅ declaration ONLY

    private:
        void handle_command(const wbp_command_ipc_t &cmd);
        bool validate(const wbp_command_ipc_t &cmd) const;

    private:
        StateMachine &state_machine_;
        CommandExecutor &executor_;
        const PrintedVolume &printed_volume_;

        /* std::optional<PauseSnapshot> pause_snapshot_; */

        int shm_fd_{-1};
        wbp_command_ipc_t *shm_ptr_{nullptr};

        uint32_t last_seen_seq_{0};
    };

} // namespace firmware
