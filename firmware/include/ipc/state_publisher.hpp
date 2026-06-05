// start of file: firmware/include/ipc/state_publisher.hpp
#pragma once

#include "ipc/machine_state_ipc.h"
#include "firmware/machine_state.hpp"
#include "ipc/command_ipc.h"

namespace firmware::ipc
{

    // Initializes shared memory and returns pointer
    wbp_machine_state_ipc_t *init_ipc_state();
    wbp_command_ipc_t *init_ipc_command();

    // Publishes current machine state into shared memory
    void publish_ipc_state(const firmware::MachineState &state);

} // namespace firmware::ipc

// end of file: firmware/include/ipc/state_publisher.hpp