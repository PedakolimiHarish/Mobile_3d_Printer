#pragma once

#include "firmware/motion/motion_controller.hpp"

namespace firmware {

class MotionControllerStub final : public MotionController {
public:
    MotionControllerStub() = default;

    MotionResult submit(const MotionCommand& cmd) override;
    MotionResult poll() override;
    void emergency_stop() override;
    bool is_ready() const override;

private:
    bool busy_{false};
};

} // namespace firmware
