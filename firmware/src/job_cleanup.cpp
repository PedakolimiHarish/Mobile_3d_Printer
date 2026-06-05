#include "firmware/job_cleanup.hpp"
#include "firmware/job_ingress_state.hpp"
#include "firmware/execution_context.hpp"
#include "firmware/machine_state.hpp"
#include "firmware/job.hpp"

namespace firmware
{

    extern JobIngressState &get_job_ingress_state();
    extern ExecutionContext &get_execution_context();
    extern MachineState &get_machine_state();
    extern Job g_active_job;

    void cleanup_completed_job()
    {
        auto &ingress = get_job_ingress_state();
        auto &exec = get_execution_context();
        auto &ms = get_machine_state();

        // -------- Execution context reset --------
        exec.phase = ExecutionPhase::IDLE;
        exec.cursor = 0;
        exec.unit_progress = 0.0;
        exec.armed = false;

        // -------- Job ingress reset --------
        ingress.job_loaded = false;
        ingress.job_id = 0;
        ingress.schema_version = 0;
        ingress.job_hash[0] = '\0';
        ingress.last_rejection[0] = '\0';

        // -------- Active job reset --------
        g_active_job = Job{};

        // -------- Machine returns to READY --------
        ms.system = SystemState::READY;
    }

    void free_job_segments(Job &job)
    {
        if (job.segments)
        {
            delete[] job.segments;
            job.segments = nullptr;
        }
    }
} // namespace firmware
