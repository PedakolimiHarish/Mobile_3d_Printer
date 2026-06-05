#pragma once
#include <cstdint>

namespace firmware {

struct PauseSnapshot {
    uint32_t job_id{0};
    uint32_t layer{0};
    uint32_t segment{0};
    double intra_progress{0.0};
    uint32_t volume_checksum{0};
};

}
