#include "firmware/anchor_types.hpp"

namespace firmware
{
    static ActiveAnchorState g_anchor_state;

    ActiveAnchorState &get_anchor_state()
    {
        return g_anchor_state;
    }
}
