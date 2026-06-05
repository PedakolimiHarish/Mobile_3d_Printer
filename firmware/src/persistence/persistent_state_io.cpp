#include "firmware/persistence/persistent_state_io.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cstdint>

namespace firmware {

// ONE declaration, inside the namespace

static constexpr const char* STATE_PATH = "/var/lib/wbp/persistent_state.bin";
static constexpr const char* STATE_TMP  = "/var/lib/wbp/persistent_state.tmp";

static uint32_t crc32(const void* data, size_t len)
{
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < len; ++i) {
        crc ^= bytes[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320u;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

PersistedStateStatus load_persistent_state(PersistentState& out)
{
    int fd = open(STATE_PATH, O_RDONLY);
    if (fd < 0)
        return PersistedStateStatus::INVALID;

    ssize_t r = read(fd, &out, sizeof(PersistentState));
    close(fd);
    if (r != sizeof(PersistentState))
        return PersistedStateStatus::INVALID;

    if (out.magic != PERSISTENCE_MAGIC)
        return PersistedStateStatus::INVALID;

    if (out.version != PERSISTENCE_VERSION)
        return PersistedStateStatus::VERSION_MISMATCH;

    PersistentState tmp{};
    std::memcpy(&tmp, &out, sizeof(PersistentState));
    tmp.crc32 = 0;

    uint32_t computed =
        crc32(&tmp, sizeof(PersistentState) - sizeof(uint32_t));
    ;

    if (computed != out.crc32)
        return PersistedStateStatus::CHECKSUM_MISMATCH;

    return PersistedStateStatus::VALID;
}

bool store_persistent_state(const PersistentState& state)
{
    PersistentState copy{};
    std::memcpy(&copy, &state, sizeof(PersistentState));

    copy.magic = PERSISTENCE_MAGIC;
    copy.version = PERSISTENCE_VERSION;
    copy.crc32 =
        crc32(&copy, sizeof(PersistentState) - sizeof(uint32_t));

    int fd = open(STATE_TMP, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0)
        return false;

    if (write(fd, &copy, sizeof(PersistentState)) != sizeof(PersistentState)) {
        close(fd);
        return false;
    }
    fsync(fd);
    close(fd);

    return rename(STATE_TMP, STATE_PATH) == 0;
}

void clear_persistent_state()
{
    unlink(STATE_PATH);
}

} // namespace firmware
