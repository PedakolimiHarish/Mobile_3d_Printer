#pragma once

#include "firmware/job_ingress_state.hpp"
#include "firmware/job_store.hpp"
#include "firmware/execution_context.hpp"
#include "firmware/machine_state.hpp"

namespace firmware
{

    bool activate_job_execution(
        JobIngressState &ingress,
        JobStore &store,
        MachineState &ms,
        ExecutionContext &exec_ctx);

} // namespace firmware
