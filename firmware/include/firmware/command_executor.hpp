// command_executor.hpp
#pragma once
#include "machine_state.hpp"
#include "printed_volume.hpp"
#include "execution_context.hpp"
#include "pause_snapshot.hpp"
#include "command.hpp"

namespace firmware {

enum class CommandResult : uint8_t {
    EXECUTED,
    BLOCKED
};

class CommandExecutor {
public:
    CommandExecutor(MachineState&, PrintedVolume&, ExecutionContext&);

    CommandResult execute(const Command&);
    void pause(PauseSnapshot&);
    bool resume(const PauseSnapshot&);

private:
    MachineState& state;
    PrintedVolume& volume;
    ExecutionContext& ctx;
};

}
