// firmware/src/printed_volume.cpp
#include "firmware/printed_volume.hpp"

namespace firmware
{

    PrintedVolume::PrintedVolume(uint32_t x, uint32_t y, double res)
        : sx(x),
          sy(y),
          resolution(res),
          height(x * y, 0.0f),
          volume_checksum(0)
    {
    }

    std::size_t PrintedVolume::idx(uint32_t x, uint32_t y) const
    {
        return static_cast<std::size_t>(y) * sx + x;
    }

    void PrintedVolume::commit(uint32_t x, uint32_t y, float z)
    {
        auto &h = height[idx(x, y)];
        if (z > h)
        {
            h = z;
            volume_checksum++;
        }
    }

    float PrintedVolume::max_height(uint32_t x, uint32_t y) const
    {
        return height[idx(x, y)];
    }

    bool PrintedVolume::lateral_safe(float z) const
    {
        for (float h : height)
        {
            if (h >= z)
                return false;
        }
        return true;
    }

} // namespace firmware
