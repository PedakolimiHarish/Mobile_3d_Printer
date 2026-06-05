// start of file: ros2_ws/src/wbp_firmware_bridge/src/ipc_command_writer.cpp

#include "wbp_firmware_bridge/ipc_command_writer.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>

namespace wbp_firmware_bridge
{

    bool IpcCommandWriter::connect()
    {
        if (shm_ptr_)
        {
            return true;
        }

        shm_fd_ = shm_open("/wbp_command_channel", O_RDWR, 0666);
        if (shm_fd_ < 0)
        {
            return false;
        }

        shm_ptr_ = static_cast<wbp_command_ipc_t *>(
            mmap(nullptr, sizeof(wbp_command_ipc_t),
                 PROT_READ | PROT_WRITE,
                 MAP_SHARED, shm_fd_, 0));

        return shm_ptr_ != MAP_FAILED;
    }

    bool IpcCommandWriter::send(wbp_command_ipc_t cmd)
    {
        if (!shm_ptr_)
        {
            return false;
        }

        std::memcpy(shm_ptr_, &cmd, sizeof(cmd));
        return true;
    }

} // namespace wbp_firmware_bridge

// end of file: ros2_ws/src/wbp_firmware_bridge/src/ipc_command_writer.cpp