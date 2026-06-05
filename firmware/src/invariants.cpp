// start of firmware/src/invariants.cpp
#include "firmware/invariants.hpp"
#include <iostream>
#include <cassert>

namespace firmware
{
    void enforce_invariants(const MachineState &ms,
                            const ExecutionContext &ctx)
    {
        if (ms.system != SystemState::EXECUTING_PRINT)
        {
            assert(ctx.phase != ExecutionPhase::RUNNING);
        }
    }

    void enforce_invariants(MachineState &ms,
                            ExecutionContext &ctx)
    {
        // I-03 READY is quiescent
        if (ms.system == SystemState::READY &&
            ctx.phase != ExecutionPhase::IDLE)
        {
            std::cerr << "[INV] READY with active execution — resetting\n";
            ctx.reset_soft();
        }

        // I-04 PAUSED is stable
        if (ms.system == SystemState::PAUSED &&
            ctx.phase != ExecutionPhase::PAUSED)
        {
            std::cerr << "[INV] PAUSED mismatch — forcing PAUSED\n";
            ctx.phase = ExecutionPhase::PAUSED;
        }

        // I-06 Abort is terminal
        if (ms.system == SystemState::ABORTING)
        {
            ctx.reset_soft();
        }

        // I-07 Fault dominates
        if (ms.system == SystemState::FAULT)
        {
            ctx.reset_soft();
        }
    }

    void enforce_execution_ownership(
        const MachineState &,
        const ExecutionContext &exec)
    {
        // RUNNING implies ownership
        if (exec.phase == ExecutionPhase::RUNNING)
        {
            assert(exec.owned_by_executor &&
                   "RUNNING execution without executor ownership");
        }

        // Not RUNNING implies no ownership
        if (exec.phase != ExecutionPhase::RUNNING)
        {
            assert(!exec.owned_by_executor &&
                   "Non-RUNNING execution with executor ownership");
        }
    }
} // namespace firmware

// end of firmware/src/invariants.cpp