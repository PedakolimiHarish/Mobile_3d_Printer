// start of file: firmware/src/segment_runner.cpp
#include "firmware/segment_runner.hpp"

namespace firmware
{

    SegmentRunner::SegmentRunner(
        ExecutionContext &ctx,
        MachineState &ms)
        : ctx_(ctx), machine_state_(ms)
    {
    }

    ExecutionResult SegmentRunner::step()
    {
        // ------------------------------------------------------------
        // Execution only allowed during EXECUTING_PRINT
        // ------------------------------------------------------------
        if (machine_state_.system != SystemState::EXECUTING_PRINT)
            return ExecutionResult::BLOCKED;

        // ------------------------------------------------------------
        // Completed execution stays completed
        // ------------------------------------------------------------
        if (ctx_.phase == ExecutionPhase::COMPLETED)
            return ExecutionResult::COMPLETED;

        // ------------------------------------------------------------
        // PAUSED execution does nothing
        // ------------------------------------------------------------
        if (ctx_.phase == ExecutionPhase::PAUSED)
            return ExecutionResult::BLOCKED;

        // ------------------------------------------------------------
        // RUNNING execution advances exactly one unit
        // (stub behavior for now)
        // ------------------------------------------------------------
        if (ctx_.phase == ExecutionPhase::RUNNING)
        {
            return ExecutionResult::PROGRESSED;
        }

        return ExecutionResult::BLOCKED;
    }

} // namespace firmware

// end of file: firmware/src/segment_runner.cpp