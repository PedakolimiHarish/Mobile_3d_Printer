#pragma once
#include "firmware/extruder/extruder_types.hpp"

namespace firmware {

class ExtruderController {
public:
    virtual ~ExtruderController() = default;

    virtual ExtruderResult submit(ExtruderCommand cmd) = 0;
    virtual ExtruderResult poll() = 0;

    virtual void emergency_stop() = 0;
    virtual bool is_ready() const = 0;
};

} // namespace firmware
