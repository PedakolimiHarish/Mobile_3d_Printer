// start of ros2_ws/src/wbp_interfaces/include/wbp_interfaces/ipc/command_ipc.h
#pragma once

#include <stdint.h>

#define WBP_COMMAND_SHM_NAME "/wbp_command_channel"
#define WBP_LOCALIZATION_FREE 0
#define WBP_LOCALIZATION_ANCHORED 1
#define WBP_COMMAND_IPC_ABI_VERSION 1

typedef enum
{
    WBP_CMD_NONE = 0,
    WBP_CMD_START = 1,
    WBP_CMD_PAUSE = 2,
    WBP_CMD_RESUME = 3,
    WBP_CMD_ABORT = 4,
    WBP_CMD_SET_LOCALIZATION_MODE = 5,
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
    uint8_t localization_mode;

} wbp_command_ipc_t;

// end of ros2_ws/src/wbp_interfaces/include/wbp_interfaces/ipc/command_ipc.h