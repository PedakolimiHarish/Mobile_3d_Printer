// firmware/include/firmware/printed_volume.hpp
#pragma once

#include <vector>
#include <cstdint>

namespace firmware
{

    struct PrintedVolume
    {
        uint32_t sx;
        uint32_t sy;
        double resolution;
        std::vector<float> height;
        uint32_t volume_checksum{0};

        // Constructor declaration only
        PrintedVolume(uint32_t x, uint32_t y, double res);

        std::size_t idx(uint32_t x, uint32_t y) const;

        // Commit declaration only
        void commit(uint32_t x, uint32_t y, float z);

        float max_height(uint32_t x, uint32_t y) const;

        bool lateral_safe(float z) const;
    };

} // namespace firmware
