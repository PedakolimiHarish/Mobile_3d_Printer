#pragma once

#include "firmware/safety/safety_state.hpp"

namespace firmware {

SafetyInputs evaluate_safety(const SafetyInputs& raw);

}
