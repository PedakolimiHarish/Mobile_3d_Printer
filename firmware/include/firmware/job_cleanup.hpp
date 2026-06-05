#pragma once
#include "firmware/job.hpp"

namespace firmware
{
    void cleanup_completed_job();
    void free_job_segments(Job &job);
}
