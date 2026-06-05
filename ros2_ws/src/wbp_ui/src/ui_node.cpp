// start of file: ros2_ws/src/wbp_ui/src/ui_node.cpp
#include "wbp_ui/ui_node.hpp"

#include <chrono>

#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>

#include "wbp_interfaces/srv/start_job.hpp"

using namespace std::chrono_literals;

namespace wbp_ui
{

  UiNode::UiNode(MainWindow *window)
      : Node("wbp_ui"), window_(window)
  {
    // ✅ CREATE CLIENTS FIRST
    start_client_ = create_client<wbp_interfaces::srv::StartJob>("/firmware/start_job");
    pause_client_ = create_client<std_srvs::srv::Trigger>("/firmware/pause");
    resume_client_ = create_client<std_srvs::srv::Trigger>("/firmware/resume");
    abort_client_ = create_client<std_srvs::srv::Trigger>("/firmware/abort");

    /* ---------------- Firmware state subscription ---------------- */
    fw_sub_ = create_subscription<wbp_interfaces::msg::MachineState>(
        "/machine_state", 10,
        [this](const wbp_interfaces::msg::MachineState &msg)
        {
          fw_state_ = msg;
          has_fw_state_ = true;

          /* RCLCPP_INFO(this->get_logger(),
                      "[UI] system_state=%u job_id=%u",
                      msg.system_state,
                      msg.job_id); */

          ui_policy_.observe_firmware_state(msg);

          update_ui();
        });

    /* ---------------- Alignment status subscription ---------------- */
    align_sub_ = create_subscription<wbp_sim_interfaces::msg::AlignmentStatus>(
        "/localization/alignment_status", 10,
        [this](const wbp_sim_interfaces::msg::AlignmentStatus &msg)
        {
          alignment_ = msg;
          has_alignment_ = true;
          update_ui();
        });

    /* ===================== UI → ROS COMMANDS ===================== */

    /* ---------- START ---------- */
    QObject::connect(window_, &MainWindow::startRequested, [this]()
                     {
  if (!start_client_->wait_for_service(100ms))
    return;

  auto req = std::make_shared<wbp_interfaces::srv::StartJob::Request>();
  start_client_->async_send_request(req);

  ui_policy_.mark_command_sent(WBP_CMD_START); });

    /* ---------- PAUSE ---------- */
    QObject::connect(window_, &MainWindow::pauseRequested, [this]()
                     {
 if (!pause_client_->wait_for_service(100ms))
  return;

auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
pause_client_->async_send_request(req);

  ui_policy_.mark_command_sent(WBP_CMD_PAUSE); });

    /* ---------- RESUME ---------- */
    QObject::connect(window_, &MainWindow::resumeRequested, [this]()
                     {
    if (!resume_client_->wait_for_service(100ms))
  return;

auto req =
    std::make_shared<std_srvs::srv::Trigger::Request>();

resume_client_->async_send_request(req);

    ui_policy_.mark_command_sent(WBP_CMD_RESUME); });

    /* ---------- ABORT ---------- */
    QObject::connect(window_, &MainWindow::abortRequested, [this]()
                     {
    if (!abort_client_->wait_for_service(100ms))
  return;

auto req =
    std::make_shared<std_srvs::srv::Trigger::Request>();

abort_client_->async_send_request(req);

    ui_policy_.mark_command_sent(WBP_CMD_ABORT); });

    /* ---------- LOAD G-CODE ---------- */
    QObject::connect(window_, &MainWindow::loadGcodeRequested,
                     [this](const QString &path)
                     {
                       RCLCPP_INFO(this->get_logger(),
                                   "[UI] Selected G-code: %s",
                                   path.toStdString().c_str());

                       auto pub = this->create_publisher<std_msgs::msg::String>(
                           "/gcode_file", 10);

                       std_msgs::msg::String msg;
                       msg.data = path.toStdString();

                       pub->publish(msg);
                     });
  }

  /* ================================================================
   * UI UPDATE
   * ================================================================ */
  void UiNode::update_ui()
  {
    if (!has_fw_state_)
      return;

    // Default alignment to "not valid" if never received
    wbp_sim_interfaces::msg::AlignmentStatus align{};
    align.status = has_alignment_ ? alignment_.status : 0;

    // ui_policy_.observe_firmware_state(fw_state_);

    auto ui = compute_ui_state(
        fw_state_,
        align,
        ui_policy_.is_command_pending());

    /* RCLCPP_INFO(this->get_logger(),
                "[UI DEBUG] updating UI for state=%u",
                fw_state_.system_state); */

    window_->updateUi(ui);
  }

} // namespace wbp_ui

// end of file: ros2_ws/src/wbp_ui/src/ui_node.cpp