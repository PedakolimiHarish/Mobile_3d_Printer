#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define WBP_IPC_MAGIC 0x57425031 // "WBP1"

    typedef struct
    {
        uint32_t magic;
        uint32_t sequence;

        uint8_t system_state;
        uint8_t safety_state;
        uint8_t paused;
        uint8_t _pad;

        uint32_t job_id;
        uint32_t layer;
        uint32_t segment;

        // ================================
        // Phase 11 — Job Ingress (Read-Only)
        // ================================

        uint8_t job_loaded;     // 0 or 1
        uint64_t loaded_job_id; // ingress job identity
        uint32_t loaded_job_schema_version;
        char loaded_job_hash[64];
        char last_job_rejection[128];
        uint32_t last_command_seq;
        uint8_t last_command_outcome;

    } wbp_machine_state_ipc_t;

#ifdef __cplusplus
}
#endif
