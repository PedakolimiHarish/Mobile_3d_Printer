#pragma once

#include "firmware/execution_result.hpp"
#include "firmware/execution_context.hpp"
#include "firmware/machine_state.hpp"

namespace firmware
{

    class SegmentRunner
    {
    public:
        SegmentRunner(ExecutionContext &ctx, MachineState &ms);

        // Called once per firmware loop
        ExecutionResult step();

    private:
        ExecutionContext &ctx_;
        MachineState &machine_state_;
    };

} // namespace firmware
