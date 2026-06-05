// start of file: firmware/include/firmware/segment.hpp

#pragma once
#include <cstdint>

namespace firmware
{

    /**
     * Segment
     *
     * Atomic unit of execution.
     * One execution_step() processes exactly ONE Segment.
     */
    struct Segment
    {

        // Layer index this segment belongs to
        uint32_t layer{0};

        // Index within the layer (for diagnostics only)
        uint32_t index_in_layer{0};

        // Target position
        double x{0.0};
        double y{0.0};
        double z{0.0};

        // Extrusion amount for this segment
        double extrusion{0.0};

        double feedrate{0.0};
    };

} // namespace firmware
