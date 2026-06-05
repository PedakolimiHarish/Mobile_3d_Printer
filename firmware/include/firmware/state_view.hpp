#pragma once
#include "firmware/machine_state.hpp"
#include "firmware/execution_context.hpp"

namespace firmware {

struct StateView {
    const MachineState& state;
    const ExecutionContext& exec;

    bool paused() const {
        return exec.paused;
    }
};

}
