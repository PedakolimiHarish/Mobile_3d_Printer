// start of file: ros2_ws/src/wbp_supervisor/src/command_policy.cpp

#include "wbp_supervisor/command_policy.hpp"
#include "wbp_interfaces/msg/machine_state.hpp"

namespace wbp_supervisor
{

    CommandPolicy::CommandPolicy() = default;

    bool CommandPolicy::can_send(wbp_command_type_t cmd,
                                 const wbp_interfaces::msg::MachineState &state) const
    {
        if (active_command_.has_value())
            return false;

        // Optional: pre-filter obvious illegal requests
        (void)cmd;
        (void)state;

        return true;
    }

    void CommandPolicy::mark_sent(wbp_command_type_t cmd,
                                  const wbp_interfaces::msg::MachineState &pre_state)
    {
        active_command_ = cmd;
        pre_state_ = pre_state;
    }

    CommandOutcome CommandPolicy::evaluate(
        const wbp_interfaces::msg::MachineState &current_state)
    {
        if (!active_command_.has_value() || !pre_state_.has_value())
            return CommandOutcome::UNRESOLVED;

        // FAULT always wins
        /*     if (current_state.system_state ==
                wbp_interfaces::msg::MachineState::SYSTEM_FAULT)
            {
                active_command_.reset();
                pre_state_.reset();
                return CommandOutcome::FAULTED;
            } */

        // State changed → ACCEPTED
        if (current_state != *pre_state_)
        {
            active_command_.reset();
            pre_state_.reset();
            return CommandOutcome::ACCEPTED;
        }

        // No change yet → still unresolved
        return CommandOutcome::UNRESOLVED;
    }

    bool CommandPolicy::command_in_flight() const
    {
        return active_command_.has_value();
    }

}
