// start of file: firmware/src/ipc/job_ingress_ipc.cpp
#include "ipc/job_ingress_ipc.hpp"
#include "firmware/job.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace firmware::ipc
{

    static constexpr const char *SHM_NAME = "/wbp_job_ingress";
    static wbp_job_ingress_ipc_t *g_job_ipc = nullptr;
    static uint32_t last_sequence_ = 0;

    wbp_job_ingress_ipc_t *init_ipc_job_ingress()
    {
        int fd = shm_open(WBP_JOB_SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (fd < 0)
        {
            perror("shm_open job ingress failed");
            return nullptr;
        }

        if (ftruncate(fd, sizeof(wbp_job_ingress_ipc_t)) < 0)
        {
            perror("ftruncate failed");
            return nullptr;
        }

        void *ptr = mmap(nullptr,
                         sizeof(wbp_job_ingress_ipc_t),
                         PROT_READ | PROT_WRITE,
                         MAP_SHARED,
                         fd,
                         0);

        if (ptr == MAP_FAILED)
        {
            perror("mmap failed");
            return nullptr;
        }

        // 🔥 FIX: assign global pointer
        g_job_ipc = static_cast<wbp_job_ingress_ipc_t *>(ptr);

        if (g_job_ipc->magic != WBP_JOB_IPC_MAGIC)
        {
            std::memset(g_job_ipc, 0, sizeof(*g_job_ipc));
            g_job_ipc->magic = WBP_JOB_IPC_MAGIC;
        }

        std::cerr << "[IPC] Job ingress IPC initialized\n";

        return g_job_ipc;
    }

    wbp_job_ingress_ipc_t *get_job_ipc()
    {
        return g_job_ipc;
    }

    bool poll_job_ipc()
    {
        if (!g_job_ipc)
            return false;

        if (g_job_ipc->magic != WBP_JOB_IPC_MAGIC)
            return false;

        if (g_job_ipc->sequence == last_sequence_)
            return false;

        last_sequence_ = g_job_ipc->sequence;

        std::cerr << "[FW] Job IPC seq=" << g_job_ipc->sequence
                  << " size=" << g_job_ipc->size << std::endl;

        return true;
    }

    extern firmware::Job g_active_job;

} // namespace firmware::ipc
