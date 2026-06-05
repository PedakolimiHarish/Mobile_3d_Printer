#pragma once
#include "firmware/persistence/persistent_state.hpp"
#include "firmware/persistence/persistence_types.hpp"

namespace firmware
{

    PersistedStateStatus load_persistent_state(PersistentState &out);
    bool store_persistent_state(const PersistentState &state);
    void clear_persistent_state();

} // namespace firmware
