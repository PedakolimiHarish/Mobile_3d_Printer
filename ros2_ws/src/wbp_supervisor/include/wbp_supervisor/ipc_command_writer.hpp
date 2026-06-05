#pragma once

#include <cstdint>
#include <optional>

#include "wbp_interfaces/ipc/command_ipc.h"

namespace wbp_supervisor
{

class IPCCommandWriter
{
public:
    IPCCommandWriter();

    bool is_connected() const;

    bool send_command(wbp_command_type_t command);

private:
    uint64_t compute_checksum(const wbp_command_ipc_t& cmd) const;

private:
    int shm_fd_{-1};
    wbp_command_ipc_t* shm_ptr_{nullptr};

    uint32_t seq_{0};
};

}
