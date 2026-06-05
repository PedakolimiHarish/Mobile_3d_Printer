#pragma once
#include <cstdint>

namespace firmware
{

    enum class ExecutionResult : uint8_t
    {
        PROGRESSED, // One execution unit advanced
        BLOCKED,    // Cannot execute now
        COMPLETED,  // Job finished successfully
        FAULTED,    // Execution failure
        STARVED     // No progress for too long
    };

} // namespace firmware
