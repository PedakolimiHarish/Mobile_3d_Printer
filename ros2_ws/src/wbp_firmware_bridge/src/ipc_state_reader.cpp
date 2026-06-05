#include "wbp_firmware_bridge/ipc_state_reader.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>

namespace wbp_firmware_bridge {

bool IpcStateReader::connect() {
    if (shm_ptr_) {
        return true;
    }

    shm_fd_ = shm_open("/wbp_machine_state", O_RDONLY, 0444);
    if (shm_fd_ < 0) {
        return false;
    }

    shm_ptr_ = static_cast<wbp_machine_state_ipc_t*>(
        mmap(nullptr, sizeof(wbp_machine_state_ipc_t),
             PROT_READ, MAP_SHARED, shm_fd_, 0));

    return shm_ptr_ != MAP_FAILED;
}

bool IpcStateReader::read(wbp_machine_state_ipc_t& out) {
    if (!shm_ptr_) {
        return false;
    }

    std::memcpy(&out, shm_ptr_, sizeof(out));
    return true;
}

} // namespace wbp_firmware_bridge
