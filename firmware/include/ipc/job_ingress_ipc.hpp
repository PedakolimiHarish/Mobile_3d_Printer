// start of file: firmware/include/ipc/job_ingress_ipc.hpp
#pragma once

#include "ipc/job_ingress_ipc.h"

namespace firmware::ipc
{

    // Create + map shared memory
    wbp_job_ingress_ipc_t *init_ipc_job_ingress();

    // Access already-mapped region
    wbp_job_ingress_ipc_t *get_job_ipc();

    void process_job_segments();

} // namespace firmware::ipc
