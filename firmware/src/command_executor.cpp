// start of file: firmware/src/command_executor.cpp
#include "firmware/command_executor.hpp"
#include "firmware/safe_travel_rules.hpp"
#include "firmware/machine_state.hpp"

namespace firmware
{

    CommandExecutor::CommandExecutor(
        MachineState &s,
        PrintedVolume &v,
        ExecutionContext &c) : state(s), volume(v), ctx(c) {}

    CommandResult CommandExecutor::execute(const Command &cmd)
    {
        const auto &oa = state.operator_actions;

        // 🔒 Phase 31.B — gate execution commands only
        if (!oa.can_start &&
            (cmd.type == CommandType::JOB_START))
        {
            return CommandResult::BLOCKED;
        }

        if (ctx.phase == ExecutionPhase::PAUSED)
        {
            return CommandResult::BLOCKED;
        }

        switch (cmd.type)
        {
        case CommandType::JOB_START:
            state.system = SystemState::EXECUTING_PRINT;
            return CommandResult::EXECUTED;

        case CommandType::SET_LAYER:
            state.job.layer = cmd.set_layer.layer;
            return CommandResult::EXECUTED;

        case CommandType::PRINT_SEGMENT:
            if (!SafeTravelRules::can_descend(
                    state, volume,
                    cmd.print.x, cmd.print.y,
                    cmd.print.z))
                return CommandResult::BLOCKED;

            volume.commit(cmd.print.x, cmd.print.y, cmd.print.z);
            state.job.segment = cmd.print.seg;
            return CommandResult::EXECUTED;

        case CommandType::JOB_END:
            state.system = SystemState::IDLE;
            return CommandResult::EXECUTED;
        }

        return CommandResult::BLOCKED;
    }

    void CommandExecutor::pause(PauseSnapshot &snap)
    {
        // Capture job identity from MachineState, not ExecutionContext
        snap.job_id = state.job.job_id;
        snap.layer = state.job.layer;
        snap.segment = state.job.segment;

        // Capture execution progress from ExecutionContext
        snap.intra_progress = ctx.unit_progress;

        // Capture printed volume checksum
        snap.volume_checksum = volume.volume_checksum;

        // Transition execution phase
        ctx.phase = ExecutionPhase::PAUSED;
    }

    bool CommandExecutor::resume(const PauseSnapshot &snap)
    {
        if (snap.volume_checksum != volume.volume_checksum)
            return false;
        ctx.phase = ExecutionPhase::RUNNING;
        ctx.unit_progress = snap.intra_progress;
        return true;
    }

}

// end of file: firmware/src/command_executor.cpp