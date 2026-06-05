// start of file: firmware/src/ipc/state_publisher.cpp
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

#include "ipc/state_publisher.hpp"
#include "ipc/command_ipc.h"

#include "firmware/job_ingress_state.hpp"

namespace firmware::ipc
{

    static constexpr const char *SHM_NAME = "/wbp_machine_state";
    static wbp_machine_state_ipc_t *g_state = nullptr;

    wbp_machine_state_ipc_t *init_ipc_state()
    {
        std::cerr << "[IPC] init_ipc_state() called\n";

        int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (fd < 0)
        {
            perror("shm_open failed");
            return nullptr;
        }

        ftruncate(fd, sizeof(wbp_machine_state_ipc_t));

        void *ptr = mmap(nullptr,
                         sizeof(wbp_machine_state_ipc_t),
                         PROT_READ | PROT_WRITE,
                         MAP_SHARED,
                         fd,
                         0);

        g_state = static_cast<wbp_machine_state_ipc_t *>(ptr);
        std::memset(g_state, 0, sizeof(*g_state));
        g_state->magic = WBP_IPC_MAGIC;
        return g_state;
    }

    wbp_command_ipc_t *init_ipc_command()
    {
        std::cerr << "[IPC] init_ipc_command() called\n";

        int fd = shm_open(WBP_COMMAND_SHM_NAME,
                          O_CREAT | O_RDWR,
                          0666);
        if (fd < 0)
        {
            perror("shm_open command channel failed");
            return nullptr;
        }

        ftruncate(fd, sizeof(wbp_command_ipc_t));

        void *ptr = mmap(nullptr,
                         sizeof(wbp_command_ipc_t),
                         PROT_READ | PROT_WRITE,
                         MAP_SHARED,
                         fd,
                         0);

        if (ptr == MAP_FAILED)
        {
            perror("mmap command channel failed");
            return nullptr;
        }

        std::memset(ptr, 0, sizeof(wbp_command_ipc_t));
        return static_cast<wbp_command_ipc_t *>(ptr);
    }

    void publish_ipc_state(const firmware::MachineState &s)
    {
        /* std::cerr << "[PUB] ms@" << &s
                  << " system=" << static_cast<int>(s.system) << "\n"; */

        if (!g_state)
            return;

        g_state->sequence++;

        // Core system state
        g_state->system_state = static_cast<uint8_t>(s.system);
        g_state->safety_state = static_cast<uint8_t>(s.safety.state);
        g_state->paused = (s.system == SystemState::PAUSED);

        // Execution progress
        g_state->layer = s.job.layer;
        g_state->segment = s.job.segment;

        // === Job ingress state (ONLY SOURCE) ===
        auto &js = firmware::get_job_ingress_state();

        g_state->job_loaded = js.job_loaded;

        // 🔥 CRITICAL FIX (ADD THIS LINE)
        g_state->job_id = js.job_id;

        g_state->loaded_job_id = js.job_id;

        g_state->loaded_job_schema_version = js.schema_version;

        std::memcpy(g_state->loaded_job_hash,
                    js.job_hash,
                    sizeof(g_state->loaded_job_hash));

        std::memcpy(g_state->last_job_rejection,
                    js.last_rejection,
                    sizeof(g_state->last_job_rejection));

        // std::cerr << "[PUB] ms@" << &s << "\n";

        g_state->last_command_seq = s.last_command_seq;
        g_state->last_command_outcome =
            static_cast<uint8_t>(s.last_command_outcome);

        /* std::cerr << "[IPC PUB] job_loaded=" << g_state->job_loaded
                  << " job_id=" << g_state->job_id
                  << std::endl; */
    }

} // namespace firmware::ipc

// end of file: firmware/src/ipc/state_publisher.cpp