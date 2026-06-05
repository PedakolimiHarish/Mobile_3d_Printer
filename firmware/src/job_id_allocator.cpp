#include <atomic>
#include <cstdint>

namespace firmware {

uint64_t allocate_job_id()
{
    static std::atomic<uint64_t> id{1};
    return id++;
}

}
