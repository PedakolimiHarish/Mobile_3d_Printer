// firmware/include/firmware/invariants.hpp

#pragma once

#include "firmware/machine_state.hpp"
#include "firmware/execution_context.hpp"

namespace firmware
{

    /**
     * Enforces global execution invariants.
     * Must be called exactly once per main loop tick.
     * Never throws. May correct state.
     */
    void enforce_invariants(MachineState &ms,
                            ExecutionContext &ctx);

    void enforce_execution_ownership(
        const MachineState &ms,
        const ExecutionContext &exec);

} // namespace firmware
