#pragma once

#include <optional>

#include "wbp_interfaces/msg/machine_state.hpp"
#include "ipc/command_ipc.h"

namespace wbp_supervisor
{

enum class CommandOutcome
{
    ACCEPTED,
    REJECTED,
    FAULTED,
    UNRESOLVED
};

class CommandPolicy
{
public:
    CommandPolicy();

    bool can_send(wbp_command_type_t cmd,
                  const wbp_interfaces::msg::MachineState& state) const;

    void mark_sent(wbp_command_type_t cmd,
                   const wbp_interfaces::msg::MachineState& pre_state);

    CommandOutcome evaluate(const wbp_interfaces::msg::MachineState& current_state);

    bool command_in_flight() const;

private:
    std::optional<wbp_command_type_t> active_command_;
    std::optional<wbp_interfaces::msg::MachineState> pre_state_;
};

}
