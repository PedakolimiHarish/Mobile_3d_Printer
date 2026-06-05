// start of firmware/include/ipc/command_ipc.h
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define WBP_COMMAND_SHM_NAME "/wbp_command_channel"
#define WBP_COMMAND_IPC_ABI_VERSION 1
    /* #define WBP_CMD_SET_LOCALIZATION_MODE 0x20 */

    typedef enum
    {
        WBP_CMD_NONE = 0,
        WBP_CMD_START = 1,
        WBP_CMD_PAUSE = 2,
        WBP_CMD_RESUME = 3,
        WBP_CMD_ABORT = 4,

        WBP_CMD_SET_LOCALIZATION_MODE = 5,

        // Phase 21 — Job lifecycle finalization
        WBP_CMD_CLEAR_JOB = 6,
        // 🔒 Phase 31.C
        WBP_CMD_RECOVER = 7
    } wbp_command_type_t;

    typedef struct
    {
        uint32_t abi_version;
        uint32_t seq;

        uint32_t command;
        uint32_t flags;

        uint64_t timestamp_ns;
        uint64_t checksum;

        uint8_t localization_mode; // FREE / ANCHORED

    } wbp_command_ipc_t;

#ifdef __cplusplus
}
#endif

// end of firmware/include/ipc/command_ipc.h