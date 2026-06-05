#pragma once
#include "firmware/anchor_state.hpp"

namespace firmware
{

    class AnchorManager
    {
    public:
        const AnchorState &state() const { return state_; }

        void set_active_anchor_count(uint32_t count)
        {
            state_.active_anchor_count = count;
            state_.any_anchor_active = (count > 0);
            state_.trusted = state_.any_anchor_active;
        }

    private:
        AnchorState state_;
    };
    AnchorManager &get_anchor_manager();
} // namespace firmware
