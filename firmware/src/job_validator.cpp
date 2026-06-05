#include "firmware/job.hpp"

namespace firmware {

struct ValidationResult {
    bool ok;
    uint32_t schema_version;
    const char* reason;
};

ValidationResult validate_job(const Job& job)
{
    if (!job.valid())
        return {false, 0, "invalid_job"};

    return {true, 1, nullptr};
}

}
