#include "firmware/safety/safety_state.hpp"

namespace firmware {

SafetyInputs evaluate_safety(const SafetyInputs& raw)
{
    SafetyInputs evaluated = raw;

    if (raw.emergency_stop || raw.guard_open || !raw.power_ok) {
        evaluated.status = SafetyStatus::FAULT;
        return evaluated;
    }

    if (!raw.frame_stable || !raw.alignment_valid) {
        evaluated.status = SafetyStatus::WARNING;
        return evaluated;
    }

    evaluated.status = SafetyStatus::SAFE;
    return evaluated;
}

} // namespace firmware
