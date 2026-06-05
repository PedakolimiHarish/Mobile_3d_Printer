#pragma once

#include <rclcpp/rclcpp.hpp>
#include <wbp_interfaces/msg/machine_state.hpp>
#include <wbp_sim_interfaces/msg/alignment_status.hpp>
#include <wbp_interfaces/srv/lifecycle_command.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <wbp_interfaces/srv/start_job.hpp>

#include "wbp_ui/ui_policy.hpp"
#include "wbp_ui/main_window.hpp"

namespace wbp_ui
{

  class UiNode : public rclcpp::Node
  {
  public:
    explicit UiNode(MainWindow *window);

  private:
    void update_ui();
    void send_command(uint8_t cmd);

    MainWindow *window_;

    bool has_fw_state_{false};
    bool has_alignment_{false};

    wbp_interfaces::msg::MachineState fw_state_;
    wbp_sim_interfaces::msg::AlignmentStatus alignment_;

    UiPolicy ui_policy_;

    rclcpp::Client<wbp_interfaces::srv::StartJob>::SharedPtr start_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr abort_client_;

    rclcpp::Subscription<wbp_interfaces::msg::MachineState>::SharedPtr fw_sub_;
    rclcpp::Subscription<wbp_sim_interfaces::msg::AlignmentStatus>::SharedPtr align_sub_;
    rclcpp::Client<wbp_interfaces::srv::LifecycleCommand>::SharedPtr cmd_client_;
  };

} // namespace wbp_ui
