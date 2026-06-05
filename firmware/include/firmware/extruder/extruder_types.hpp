#pragma once
#include <cstdint>

namespace firmware {

enum class ExtruderCommand : uint8_t {
    EXTRUDE_ON,
    EXTRUDE_OFF
};

enum class ExtruderResult : uint8_t {
    ACCEPTED,
    IN_PROGRESS,
    COMPLETED,
    REJECTED,
    FAULTED
};

} // namespace firmware
