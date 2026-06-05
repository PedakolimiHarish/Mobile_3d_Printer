#pragma once
#include "wbp_interfaces/ipc/machine_state_ipc.h"

class IpcReader {
public:
    wbp_machine_state_ipc_t* map();
};
