// start of file: include/firmware/motion/motion_controller.hpp
#pragma once
#include "firmware/motion/motion_types.hpp"

namespace firmware
{

    class MotionController
    {
    public:
        virtual ~MotionController() = default;

        // Submit a new command (only when idle)
        virtual MotionResult submit(const MotionCommand &cmd) = 0;

        // Poll current command state
        virtual MotionResult poll() = 0;

        // Immediate halt (fault or estop)
        virtual void emergency_stop() = 0;

        // Ready to accept new commands
        virtual bool is_ready() const = 0;

        virtual void set_abort_active(bool active) = 0;

        virtual void abort() = 0;

    private:
        virtual bool is_idle() const = 0;
    };

} // namespace firmware

// end of file: include/firmware/motion/motion_controller.hpp