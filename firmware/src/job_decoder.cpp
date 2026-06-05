#include "firmware/job.hpp"
#include <vector>
#include <stdexcept>
#include <cstring>

namespace firmware
{

    Job decode_job(const std::vector<uint8_t> &blob)
    {

        std::cerr << "[DEBUG] sizeof(Segment)=" << sizeof(Segment) << "\n";

        if (blob.size() < sizeof(Segment))
            throw std::runtime_error("invalid job blob size");

        Job j{};

        constexpr size_t SEGMENT_SIZE = sizeof(Segment);
        uint32_t count = blob.size() / SEGMENT_SIZE;

        Segment *segments = new Segment[count];
        std::memcpy(segments, blob.data(), count * SEGMENT_SIZE);

        j.total_segments = count;
        j.segments = segments;

        return j;
    }

}
