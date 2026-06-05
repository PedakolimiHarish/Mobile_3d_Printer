// start of file: ros2_ws/src/wbp_interfaces/include/wbp_interfaces/ipc/segment.hpp
#pragma once
#include <cstdint>

namespace wbp_interfaces
{

    struct Segment
    {
        uint32_t layer{0};
        uint32_t index_in_layer{0};

        double x{0.0};
        double y{0.0};
        double z{0.0};

        double extrusion{0.0};

        double feedrate{0.0};
    };

}