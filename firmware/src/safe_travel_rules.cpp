#include "firmware/safe_travel_rules.hpp"

namespace firmware {

bool SafeTravelRules::can_lateral(
    const MachineState& s,
    const PrintedVolume& v,
    double z
) {
    return s.safety.state == SafetyState::OK &&
           v.lateral_safe(static_cast<float>(z));
}

bool SafeTravelRules::can_descend(
    const MachineState& s,
    const PrintedVolume& v,
    uint32_t x, uint32_t y,
    double z
) {
    return s.safety.state == SafetyState::OK &&
           z > v.max_height(x,y);
}

}