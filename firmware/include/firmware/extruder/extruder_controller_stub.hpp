#pragma once

#include "firmware/extruder/extruder_controller.hpp"

namespace firmware {

class ExtruderControllerStub final : public ExtruderController {
public:
    ExtruderControllerStub() = default;
    ~ExtruderControllerStub() override = default;

    ExtruderResult submit(ExtruderCommand cmd) override;
    ExtruderResult poll() override;
    void emergency_stop() override;
    bool is_ready() const override;

private:
    bool busy_{false};
};

} // namespace firmware
