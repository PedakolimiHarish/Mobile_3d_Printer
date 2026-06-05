// start of file: ros2_ws/src/wbp_interfaces/include/wbp_interfaces/ipc/job_ingress_ipc.h
#pragma once
#include <stdint.h>

#define WBP_JOB_SHM_NAME "/wbp_job_ingress"
#define WBP_JOB_IPC_MAGIC 0x57424A31 // "WBJ1"
#define WBP_JOB_MAX_SIZE 65536

typedef struct
{
    uint32_t magic;
    uint32_t sequence;
    uint32_t size;
    char source[64];
    uint8_t job_blob[WBP_JOB_MAX_SIZE];

    uint8_t ingress_complete; // optional signal for test mode
} wbp_job_ingress_ipc_t;
