// firmware/include/firmware/execution_orchestrator.hpp
#pragma once

#include <optional>
#include "firmware/execution_context.hpp"
#include "firmware/execution_result.hpp"
#include "firmware/state_machine.hpp"
#include "firmware/machine_state.hpp"
#include "firmware/pause_snapshot.hpp"
#include "firmware/motion/motion_controller.hpp"

namespace firmware
{

    class ExecutionOrchestrator
    {

    public:
        ExecutionOrchestrator(
            ExecutionContext &exec_ctx,
            StateMachine &sm,
            MachineState &ms,
            std::optional<PauseSnapshot> &pause_snapshot,
            MotionController &motion);

        void step(ExecutionResult last_result);

    private:
        ExecutionContext &ctx_;
        StateMachine &sm_;
        MachineState &machine_state_;
        std::optional<PauseSnapshot> &pause_snapshot_;
        MotionController &motion_;
    };

} // namespace firmware
