#pragma once

#include <optional>
#include <string>
#include <wbp_interfaces/ipc/machine_state_ipc.h>

namespace wbp_firmware_bridge {

class IpcStateReader {
public:
    bool connect();
    bool read(wbp_machine_state_ipc_t& out);

private:
    int shm_fd_{-1};
    wbp_machine_state_ipc_t* shm_ptr_{nullptr};
};

} // namespace wbp_firmware_bridge
