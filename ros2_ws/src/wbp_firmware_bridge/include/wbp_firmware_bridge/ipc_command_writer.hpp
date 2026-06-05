#pragma once

#include <wbp_interfaces/ipc/command_ipc.h>

namespace wbp_firmware_bridge {

class IpcCommandWriter {
public:
    bool connect();
    bool send(wbp_command_ipc_t cmd);

private:
    int shm_fd_{-1};
    wbp_command_ipc_t* shm_ptr_{nullptr};
};

} // namespace wbp_firmware_bridge
