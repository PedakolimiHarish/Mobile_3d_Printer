// firmware/include/firmware/job_store.hpp
#pragma once

#include <optional>
#include <string>
#include <cstdint>

#include "firmware/job.hpp"

namespace firmware
{

    class JobStore
    {
    public:
        bool has_job() const { return job_.has_value(); }

        const Job &job() const { return *job_; }

        uint64_t job_id() const { return job_id_; }
        const std::string &job_hash() const { return job_hash_; }
        uint32_t job_schema_version() const { return job_schema_version_; }

        void set_job(const Job &job,
                     uint64_t job_id,
                     std::string job_hash,
                     uint32_t schema_version)
        {
            job_ = job;
            job_id_ = job_id;
            job_hash_ = std::move(job_hash);
            job_schema_version_ = schema_version;
        }

        void clear()
        {
            job_.reset();
            job_id_ = 0;
            job_hash_.clear();
            job_schema_version_ = 0;
        }

    private:
        std::optional<Job> job_;
        uint64_t job_id_{0};
        std::string job_hash_;
        uint32_t job_schema_version_{0};
    };
    JobStore &get_job_store();

} // namespace firmware
