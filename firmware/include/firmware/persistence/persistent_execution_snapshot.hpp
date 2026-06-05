// firmware/include/firmware/persistence/persistent_execution_snapshot.hpp

#pragma once

#include "firmware/execution_context.hpp"
#include "firmware/machine_state.hpp"

#include <cstdint>
constexpr uint32_t SNAPSHOT_MAGIC = 0x57525053; // "WRPS"

namespace firmware
{

    struct PersistentExecutionSnapshot
    {
        uint32_t magic;          // MUST be SNAPSHOT_MAGIC
        uint64_t firmware_epoch; // incremented every boot
        uint64_t boot_id;
        uint64_t job_id;

        SystemState system_at_snapshot; // 🔴 ADD THIS

        ExecutionPhase phase;

        uint32_t cursor;
        double unit_progress;

        uint32_t layer;
        uint32_t segment;

        bool execution_active; // 🔴 REQUIRED

        uint32_t checksum;

        char job_hash[64];
        uint32_t job_schema_version;
    };

    bool persist_execution_snapshot(
        const PersistentExecutionSnapshot &snap);

    bool load_execution_snapshot(
        PersistentExecutionSnapshot &out);

    void clear_execution_snapshot();

} // namespace firmware
