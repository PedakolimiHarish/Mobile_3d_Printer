// firmware/src/persistence/persistent_execution_snapshot.cpp

#include "firmware/persistence/persistent_execution_snapshot.hpp"

#include <fstream>
#include <cstring>

namespace firmware
{

    // 🔒 Single fixed location (Phase 25.A: simple + deterministic)
    static const char *SNAPSHOT_PATH =
        "/var/lib/wbp/persistent_execution_snapshot.bin";

    bool persist_execution_snapshot(
        const PersistentExecutionSnapshot &snap)
    {
        std::ofstream out(SNAPSHOT_PATH, std::ios::binary | std::ios::trunc);
        if (!out)
            return false;

        out.write(reinterpret_cast<const char *>(&snap), sizeof(snap));
        return out.good();
    }

    bool load_execution_snapshot(
        PersistentExecutionSnapshot &out_snap)
    {
        std::ifstream in(SNAPSHOT_PATH, std::ios::binary);
        if (!in)
            return false;

        in.read(reinterpret_cast<char *>(&out_snap), sizeof(out_snap));
        return in.good();
    }

    void clear_execution_snapshot()
    {
        std::ofstream out(SNAPSHOT_PATH, std::ios::binary | std::ios::trunc);
        // Empty file == cleared snapshot
    }

} // namespace firmware
