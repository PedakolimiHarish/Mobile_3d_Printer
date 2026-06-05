#pragma once

#include "firmware/execution_context.hpp"
#include "firmware/execution_result.hpp"
#include "firmware/extruder/extruder_controller.hpp"

namespace firmware {

// Forward declarations only
class Job;
class MotionController;

// One deterministic execution step
ExecutionResult execution_step(
    ExecutionContext& ctx,
    const Job& job,
    MotionController& motion,
    ExtruderController& extruder
);

} // namespace firmware
