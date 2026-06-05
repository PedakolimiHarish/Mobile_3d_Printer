// start of file: firmware/src/motion/motion_controller_stub.cpp
#include "firmware/motion/motion_controller_stub.hpp"

namespace firmware
{

    MotionResult MotionControllerStub::submit(const MotionCommand &)
    {
        if (busy_)
            return MotionResult::REJECTED;

        busy_ = true;
        return MotionResult::ACCEPTED;
    }

    MotionResult MotionControllerStub::poll()
    {
        if (!busy_)
            return MotionResult::COMPLETED;

        busy_ = false;
        return MotionResult::COMPLETED;
    }

    void MotionControllerStub::emergency_stop()
    {
        busy_ = false;
    }

    bool MotionControllerStub::is_ready() const
    {
        return !busy_;
    }

} // namespace firmware

// end of file: firmware/src/motion/motion_controller_stub.cpp