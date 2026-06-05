#pragma once
#include <cstdint>

namespace firmware {

enum class SafetyStatus : uint8_t {
    SAFE,
    WARNING,
    FAULT
};

struct SafetyInputs {
    SafetyStatus status{SafetyStatus::SAFE};

    // Hard safety inputs
    bool emergency_stop{false};
    bool guard_open{false};
    bool power_ok{true};

    // Structural integrity
    bool frame_stable{true};
    bool alignment_valid{true};
};

} // namespace firmware
