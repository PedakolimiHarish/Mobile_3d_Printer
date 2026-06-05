#pragma once
#include <cstdint>
#include <cstddef>
#include <iostream>

#include "firmware/segment.hpp"

namespace firmware
{

    /**
     * Job
     *
     * Immutable execution plan.
     * Owns the ordered list of execution segments.
     *
     * A Job contains NO execution state.
     * Progress is tracked exclusively by ExecutionContext.
     */
    struct Job
    {

        // Unique job identifier
        uint32_t job_id{0};

        // Total number of layers in this job
        uint32_t total_layers{0};

        // Total number of execution segments
        uint32_t total_segments = 0;

        // Pointer to flat, ordered segment array
        // Ownership is external (e.g., loaded job buffer)
        const Segment *segments{nullptr};

        bool segments_exhausted = false;

        uint32_t received_segments = 0;
        bool ingress_completed = false;

        bool has_segment(uint32_t index) const
        {
            std::cerr << "[HAS_SEGMENT] index=" << index
                      << " received_segments=" << received_segments
                      << " total_segments=" << total_segments
                      << "\n";

            if (!segments)
            {
                return false;
            }
            return index < received_segments;
        }

        // Safety: job is valid only if segments != nullptr
        bool valid() const
        {
            return segments != nullptr && total_segments > 0;
        }
    };

} // namespace firmware
