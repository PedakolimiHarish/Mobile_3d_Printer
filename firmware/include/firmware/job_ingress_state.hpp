// firmware/include/firmware/job_ingress_state.hpp

#pragma once
#include <cstdint>

namespace firmware
{

    struct JobIngressState
    {
        bool job_loaded{false};
        uint64_t job_id{0};
        uint32_t schema_version{0};
        char job_hash[64]{};
        char last_rejection[128]{};

        void clear()
        {
            job_loaded = false;
            job_id = 0;
        }
    };

    JobIngressState &get_job_ingress_state();

}
