// start of file: ros2_ws/src/wbp_ui/src/ui_policy.cpp
#include "wbp_ui/ui_policy.hpp"

namespace wbp_ui
{

  UiState compute_ui_state(
      const wbp_interfaces::msg::MachineState &fw,
      const wbp_sim_interfaces::msg::AlignmentStatus &align,
      bool command_pending)
  {
    UiState ui;

    // HARD LOCK WHILE COMMAND PENDING
    if (command_pending)
    {
      ui.start.enabled = false;
      ui.start.reason = "Command pending";
      ui.pause.enabled = false;
      ui.resume.enabled = false;
      ui.abort.enabled = false;
      return ui;
    }

    const bool executing =
        (fw.layer > 0 || fw.segment > 0);

    // START
    if (!fw.job_loaded)
    {
      ui.start.enabled = false;
      ui.start.reason = "No job loaded";
    }
    else if (executing)
    {
      ui.start.enabled = false;
      ui.start.reason = "Already executing";
    }
    else
    {
      ui.start.enabled = true;
    }

    // PAUSE
    if (!executing)
    {
      ui.pause.enabled = false;
      ui.pause.reason = "Not executing";
    }
    else if (fw.paused)
    {
      ui.pause.enabled = false;
      ui.pause.reason = "Already paused";
    }
    else
    {
      ui.pause.enabled = true;
    }

    // RESUME
    if (!fw.paused)
    {
      ui.resume.enabled = false;
      ui.resume.reason = "Not paused";
    }
    else if (align.status != wbp_sim_interfaces::msg::AlignmentStatus::VALID)
    {
      ui.resume.enabled = false;
      ui.resume.reason = "Alignment invalid";
    }
    else
    {
      ui.resume.enabled = true;
    }

    // ABORT
    ui.abort.enabled = executing || fw.paused;

    return ui;
  }

  /* ============================================================
   * UiPolicy
   * ============================================================ */

  void UiPolicy::mark_command_sent(wbp_command_type_t type)
  {
    pending_ = PendingCommand{type};
  }

  void UiPolicy::observe_firmware_state(
      const wbp_interfaces::msg::MachineState &fw)
  {
    if (!pending_)
      return;

    // START resolves only when execution begins
    if (pending_->type == WBP_CMD_START)
    {
      if (fw.layer > 0 || fw.segment > 0)
        pending_.reset();
      return;
    }

    // PAUSE resolves when paused flips true
    if (pending_->type == WBP_CMD_PAUSE)
    {
      if (fw.paused)
        pending_.reset();
      return;
    }

    // RESUME resolves when paused clears
    if (pending_->type == WBP_CMD_RESUME)
    {
      if (!fw.paused)
        pending_.reset();
      return;
    }

    // ABORT resolves when job unloaded
    if (pending_->type == WBP_CMD_ABORT)
    {
      if (!fw.job_loaded)
        pending_.reset();
      return;
    }
  }

  bool UiPolicy::is_command_pending() const
  {
    return pending_.has_value();
  }

} // namespace wbp_ui

// end of file: ros2_ws/src/wbp_ui/src/ui_policy.cpp