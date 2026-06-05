// firmware/include/firmware/job_ingress.hpp
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace firmware
{

    struct JobSubmitResult
    {
        bool accepted{false};
        std::string rejection_reason;
        uint64_t job_id{0};
        std::string job_hash;
    };

    // 🔴 ADD THIS DECLARATION
    JobSubmitResult submit_job(const std::vector<uint8_t> &job_blob,
                               const std::string &source);

}
