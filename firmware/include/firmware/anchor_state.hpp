#pragma once
#include <cstdint>
#include "firmware/anchor_types.hpp"

namespace firmware
{
    ActiveAnchorState &get_anchor_state();

    struct AnchorState
    {
        bool any_anchor_active{false};
        uint32_t active_anchor_count{0};

        // Monotonic validity — never inferred
        bool trusted{false};
    };
}
