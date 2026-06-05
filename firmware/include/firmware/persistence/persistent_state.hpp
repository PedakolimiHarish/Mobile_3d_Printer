#pragma once
#include <cstdint>
#include "firmware/execution_context.hpp"

namespace firmware {

struct PersistentJobIdentity {
    uint64_t job_id;
    uint64_t job_hash;
};

struct PersistentState {
    uint32_t magic;
    uint32_t version;

    PersistentJobIdentity job;
    ExecutionContext execution;

    uint64_t printed_volume_checksum;

    uint32_t crc32; // must be last
};

} // namespace firmware
