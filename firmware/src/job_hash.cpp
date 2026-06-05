#include "firmware/job.hpp"
#include <cstring>

namespace firmware {

void compute_job_hash(const Job&, char out[64])
{
    std::strncpy(out, "JOB_HASH_STUB_V1", 63);
    out[63] = '\0';
}

}
