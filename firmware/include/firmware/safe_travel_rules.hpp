#pragma once
#include "machine_state.hpp"
#include "printed_volume.hpp"

namespace firmware {

struct SafeTravelRules {
    static bool can_lateral(const MachineState&, const PrintedVolume&, double);
    static bool can_descend(const MachineState&, const PrintedVolume&, uint32_t, uint32_t, double);
};

}