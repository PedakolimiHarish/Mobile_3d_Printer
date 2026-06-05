#pragma once
#include <cstdint>

namespace firmware
{

    using AnchorId = uint32_t;

    struct AnchorPose
    {
        double x;
        double y;
        double z;
        double yaw;
    };

    struct AnchorRecord
    {
        AnchorId id;
        AnchorPose world_pose;
        double max_position_error_m;
        double max_yaw_error_rad;
    };

    enum class AnchorStatus : uint8_t
    {
        NONE,
        PROPOSED,
        ACTIVE,
        INVALID
    };

    struct ActiveAnchorState
    {
        AnchorId id{0};
        AnchorPose estimated_world_pose{};
        double confidence{0.0};
        uint64_t last_verified_ms{0};
        AnchorStatus status{AnchorStatus::NONE};
    };

} // namespace firmware
