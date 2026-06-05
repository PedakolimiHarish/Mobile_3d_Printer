// start of file: firmware/src/extruder/extruder_controller_stub.cpp
#include "firmware/extruder/extruder_controller_stub.hpp"
#include <iostream>

namespace firmware
{

    ExtruderResult ExtruderControllerStub::submit(ExtruderCommand cmd)
    {
        // std::cerr << "[EXTRUDER] submit " << int(cmd) << "\n";

        if (busy_)
            return ExtruderResult::REJECTED;

        busy_ = true;
        return ExtruderResult::ACCEPTED;
    }

    ExtruderResult ExtruderControllerStub::poll()
    {
        // std::cerr << "[EXTRUDER] poll\n";

        if (!busy_)
            return ExtruderResult::COMPLETED;

        busy_ = false;
        return ExtruderResult::COMPLETED;
    }

    void ExtruderControllerStub::emergency_stop()
    {
        busy_ = false;
    }

    bool ExtruderControllerStub::is_ready() const
    {
        return !busy_;
    }

} // namespace firmware

// end of file: firmware/src/extruder/extruder_controller_stub.cpp
