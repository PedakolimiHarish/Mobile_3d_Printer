// ros2_ws/src/wbp_ui/include/wbp_ui/ui_policy.hpp
#pragma once

#include <optional>
#include "wbp_ui/ui_state.hpp"
#include "wbp_interfaces/msg/machine_state.hpp"
#include "wbp_sim_interfaces/msg/alignment_status.hpp"
#include "wbp_interfaces/ipc/command_ipc.h" // wbp_command_type_t

namespace wbp_ui
{

  UiState compute_ui_state(
      const wbp_interfaces::msg::MachineState &fw,
      const wbp_sim_interfaces::msg::AlignmentStatus &align,
      bool command_pending);

  struct PendingCommand
  {
    wbp_command_type_t type;
  };

  class UiPolicy
  {
  public:
    void mark_command_sent(wbp_command_type_t type);
    void observe_firmware_state(const wbp_interfaces::msg::MachineState &fw);
    bool is_command_pending() const;

  private:
    std::optional<PendingCommand> pending_;
    uint32_t last_seen_seq_{0};
  };

} // namespace wbp_ui
