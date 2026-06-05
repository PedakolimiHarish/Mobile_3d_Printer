// start of file: ros2_ws/src/wbp_supervisor/src/ipc_reader.cpp

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdexcept>
#include "wbp_supervisor/ipc_reader.hpp"

static constexpr const char *SHM_NAME = "/wbp_machine_state";

wbp_machine_state_ipc_t *IpcReader::map()
{
    int fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (fd < 0)
        throw std::runtime_error("shm_open failed");

    void *ptr = mmap(nullptr,
                     sizeof(wbp_machine_state_ipc_t),
                     PROT_READ,
                     MAP_SHARED,
                     fd,
                     0);

    auto *state = static_cast<wbp_machine_state_ipc_t *>(ptr);
    if (state->magic != WBP_IPC_MAGIC)
        throw std::runtime_error("Bad IPC magic");

    return state;
}
