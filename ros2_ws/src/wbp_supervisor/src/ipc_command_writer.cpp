// start of file: ros2_ws/src/wbp_supervisor/src/ipc_command_writer.cpp
#include "wbp_supervisor/ipc_command_writer.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <time.h>

namespace wbp_supervisor
{

    static uint64_t xor_checksum(const wbp_command_ipc_t &c)
    {
        return static_cast<uint64_t>(c.abi_version) ^ static_cast<uint64_t>(c.seq) ^ static_cast<uint64_t>(c.command) ^ static_cast<uint64_t>(c.flags) ^ c.timestamp_ns;
    }

    IPCCommandWriter::IPCCommandWriter()
    {
        shm_fd_ = shm_open(WBP_COMMAND_SHM_NAME, O_RDWR, 0);
        if (shm_fd_ < 0)
            return;

        shm_ptr_ = static_cast<wbp_command_ipc_t *>(
            mmap(nullptr,
                 sizeof(wbp_command_ipc_t),
                 PROT_READ | PROT_WRITE,
                 MAP_SHARED,
                 shm_fd_,
                 0));

        if (!shm_ptr_)
            return;

        // Initialize ABI version once
        shm_ptr_->abi_version = WBP_COMMAND_IPC_ABI_VERSION;
    }

    bool IPCCommandWriter::is_connected() const
    {
        return shm_ptr_ != nullptr;
    }

    bool IPCCommandWriter::send_command(wbp_command_type_t command)
    {
        if (!shm_ptr_)
            return false;

        if (command == WBP_CMD_NONE)
            return false;

        wbp_command_ipc_t cmd{};
        cmd.abi_version = WBP_COMMAND_IPC_ABI_VERSION;
        cmd.command = command;
        cmd.flags = 0;

        struct timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);
        cmd.timestamp_ns =
            static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
            static_cast<uint64_t>(ts.tv_nsec);

        cmd.seq = ++seq_;
        cmd.checksum = xor_checksum(cmd);

        // Write ordering: fields first, seq last
        std::memcpy(shm_ptr_, &cmd, sizeof(cmd));

        return true;
    }

}
