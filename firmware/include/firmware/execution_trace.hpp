// start of firmware/include/firmware/execution_trace.hpp

#pragma once
#include <iostream>
#include "firmware/execution_context.hpp"
#include "firmware/execution_result.hpp"
#include "firmware/machine_state.hpp"

namespace firmware
{
    inline const char *phase_str(ExecutionPhase p)
    {
        switch (p)
        {
        case ExecutionPhase::IDLE:
            return "IDLE";
        case ExecutionPhase::RUNNING:
            return "RUNNING";
        case ExecutionPhase::PAUSED:
            return "PAUSED";
        case ExecutionPhase::COMPLETING:
            return "COMPLETING";
        case ExecutionPhase::COMPLETED:
            return "COMPLETED";
        default:
            return "UNKNOWN";
        }
    }

    inline const char *result_str(ExecutionResult r)
    {
        switch (r)
        {
        case ExecutionResult::PROGRESSED:
            return "PROGRESSED";
        case ExecutionResult::BLOCKED:
            return "BLOCKED";
        case ExecutionResult::COMPLETED:
            return "COMPLETED";
        case ExecutionResult::FAULTED:
            return "FAULTED";
        default:
            return "UNKNOWN";
        }
    }

    inline void trace_execution(
        const ExecutionContext &ctx,
        const MachineState &ms,
        ExecutionResult result)
    {
        std::cerr
            << "[EXEC]"
            << " sys=" << static_cast<int>(ms.system)
            << " phase=" << phase_str(ctx.phase)
            << " cursor=" << ctx.cursor
            << " prog=" << ctx.unit_progress
            << " result=" << result_str(result)
            << "\n";
    }
}

// end of firmware/include/firmware/execution_trace.hpp