// start firmware/src/execution_gate.cpp

#include "firmware/execution_gate.hpp"
#include "firmware/job_ingress_state.hpp"
#include "firmware/job_store.hpp"
#include "firmware/execution_context.hpp"
#include "firmware/machine_state.hpp"
#include "firmware/anchor_manager.hpp"

#include <iostream>
namespace firmware
{

    bool activate_job_execution(
        JobIngressState &ingress,
        JobStore &,
        MachineState &ms,
        ExecutionContext &ctx)
    {
        // START path only
        if (ms.system != SystemState::EXECUTING_PRINT)
            return false;

        if (ctx.phase != ExecutionPhase::IDLE)
            return false;

        if (ctx.armed) // resume already armed
            return false;

        if (!ingress.job_loaded || !ms.job_present)
            return false;

        ctx.armed = true;
        ctx.completion_emitted = false;

        std::cerr << "[GATE] execution armed (START)\n";
        return true;
    }

}

// end of firmware/src/execution_gate.cpp