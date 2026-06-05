// firmware/include/firmware/command_outcome.hpp
#pragma once
#include <cstdint>

namespace firmware
{

    enum class CommandOutcome : uint8_t
    {
        NONE = 0,     // No outcome yet
        ACCEPTED = 1, // Command accepted and acted upon
        REJECTED = 2, // Command rejected by firmware logic
        FAULTED = 3   // Command caused or encountered a fault
    };

} // namespace firmware
