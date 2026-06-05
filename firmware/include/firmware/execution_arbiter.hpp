// firmware/include/firmware/execution_arbiter.hpp
#pragma once

namespace firmware
{

    struct ExecutionArbiter
    {
        bool armed = false;   // set by StateMachine only
        bool blocked = false; // pause / fault / safety

        bool can_execute() const
        {
            return armed && !blocked;
        }
    };

    ExecutionArbiter &get_execution_arbiter();

}
