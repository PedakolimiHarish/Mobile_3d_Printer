// start of file: firmware/include/firmware/motion/motion_types.hpp
#pragma once
#include <cstdint>

namespace firmware
{

    enum class MotionIntent : uint8_t
    {
        TRAVEL,
        DEPOSIT,
        RETRACT,
        UNRETRACT,
        WAIT_FOR_CONDITION,
    };

    struct MotionCommand
    {
        MotionIntent intent;
        double x;
        double y;
        double z;
        double feedrate;
        double extrusion;
    };

    enum class MotionResult : uint8_t
    {
        ACCEPTED,
        IN_PROGRESS,
        COMPLETED,
        REJECTED,
        FAULTED
    };

} // namespace firmware
