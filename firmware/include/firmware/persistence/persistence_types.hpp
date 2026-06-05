#pragma once
#include <cstdint>

namespace firmware {

constexpr uint32_t PERSISTENCE_MAGIC = 0x57504250; // "WBP"
constexpr uint32_t PERSISTENCE_VERSION = 1;

enum class PersistedStateStatus : uint8_t {
    VALID,
    INVALID,
    VERSION_MISMATCH,
    CHECKSUM_MISMATCH,
};

} // namespace firmware
