// start of file: ros2_ws/src/wbp_supervisor/src/ipc_job_writer.cpp
#include "wbp_supervisor/ipc_job_writer.hpp"
// #include "firmware/job.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

struct Segment
{
    // Layer index this segment belongs to
    uint32_t layer;

    // Index within the layer (for diagnostics only)
    uint32_t index_in_layer;

    // Target position
    double x;
    double y;
    double z;

    // Extrusion amount for this segment
    double extrusion;
    double feedrate;
};

IPCJobWriter::IPCJobWriter()
{
    int fd = shm_open(WBP_JOB_SHM_NAME, O_RDWR, 0666);
    if (fd < 0)
    {
        perror("shm_open job ingress failed");
        job_ipc_ = nullptr;
        return;
    }

    void *ptr = mmap(nullptr,
                     sizeof(wbp_job_ingress_ipc_t),
                     PROT_READ | PROT_WRITE,
                     MAP_SHARED,
                     fd,
                     0);

    if (ptr == MAP_FAILED)
    {
        perror("mmap job ingress failed");
        job_ipc_ = nullptr;
        return;
    }

    job_ipc_ = static_cast<wbp_job_ingress_ipc_t *>(ptr);

    if (job_ipc_->magic != WBP_JOB_IPC_MAGIC)
    {
        std::cerr << "[SUP] Job IPC not initialized by firmware\n";

        return;
    }
}

bool IPCJobWriter::write_job(
    const std::vector<uint8_t> &blob,
    const std::string &source)
{
    if (!job_ipc_)
        return false;
    if (blob.empty())
        return false;
    if (blob.size() > WBP_JOB_MAX_SIZE)
        return false;

    std::cerr << "[ROS] blob.size=" << blob.size() << "\n";

    std::memcpy(job_ipc_->job_blob, blob.data(), blob.size());
    job_ipc_->size = blob.size();
    job_ipc_->sequence++;

    std::memset(job_ipc_->source, 0, sizeof(job_ipc_->source));
    std::strncpy(job_ipc_->source, source.c_str(),
                 sizeof(job_ipc_->source) - 1);

    // 🔥🔥🔥 CRITICAL FIX
    job_ipc_->ingress_complete = 1;

    std::cerr << "[SUP WRITE] size=" << job_ipc_->size << std::endl;

    return true;
}

/* void IPCJobWriter::mark_ingress_complete()
{
    if (!job_ipc_)
        return;

    job_ipc_->ingress_complete = 1; // 🔥 THIS IS THE SIGNAL
} */

// end of file: ros2_ws/src/wbp_supervisor/src/ipc_job_writer.cpp