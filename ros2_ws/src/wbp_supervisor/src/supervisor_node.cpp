// start of file: ros2_ws/src/wbp_supervisor/src/supervisor_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <optional>
#include <thread>

#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/empty.hpp>

#include "wbp_supervisor/ipc_reader.hpp"
#include "wbp_supervisor/ipc_command_writer.hpp"
#include "wbp_supervisor/command_policy.hpp"
#include "wbp_interfaces/msg/motion_runtime_state.hpp"

#include "wbp_interfaces/msg/machine_state.hpp"
#include "wbp_interfaces/srv/job_submit.hpp"
#include "wbp_interfaces/srv/start_job.hpp"

#include "wbp_interfaces/ipc/machine_state_ipc.h"
#include "wbp_interfaces/ipc/command_ipc.h"
#include "wbp_supervisor/ipc_job_writer.hpp"

using namespace std::chrono_literals;
namespace ws = wbp_supervisor;

class SupervisorNode : public rclcpp::Node
{
public:
  SupervisorNode()
      : Node("supervisor_node")
  {
    /* ---------- IPC reader (firmware → ROS) ---------- */
    ipc_reader_ = std::make_shared<IpcReader>();
    shm_state_ = ipc_reader_->map();

    /* ---------- IPC writer (ROS → firmware) ---------- */
    command_writer_ = std::make_unique<ws::IPCCommandWriter>();
    job_writer_ = std::make_unique<IPCJobWriter>();

    /* ---------- Publisher ---------- */
    state_pub_ = this->create_publisher<wbp_interfaces::msg::MachineState>(
        "machine_state", 10);

    motion_override_pub_ =
        this->create_publisher<std_msgs::msg::Float64>(
            "/motion_override", 10);

    start_job_srv_ = this->create_service<wbp_interfaces::srv::StartJob>(
        "/firmware/start_job",
        [this](
            const std::shared_ptr<wbp_interfaces::srv::StartJob::Request>,
            std::shared_ptr<wbp_interfaces::srv::StartJob::Response> response)
        {
          // 1️⃣ trigger ProgramManager FIRST
          if (start_stream_pub_)
          {
            std_msgs::msg::Empty msg;
            start_stream_pub_->publish(msg);
          }

          start_requested_ = true;

          response->accepted = true;
          response->reason = "job start triggered";
        },
        rmw_qos_profile_services_default);

    pause_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/firmware/pause",
        [this](
            const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
          bool ok = send_pause_to_firmware();

          response->success = ok;
          response->message = ok ? "pause sent" : "pause failed";
        });

    resume_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/firmware/resume",
        [this](
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
          if (!last_motion_state_)
          {
            response->success = false;
            response->message = "No motion runtime state yet";
            return;
          }

          // Motion must be IDLE and NORMAL
          if (last_motion_state_->exec_state == 3) // Fault
          {
            response->success = false;
            response->message = "Motion executor not in safe state";
            return;
          }

          send_command(WBP_CMD_RESUME);

          std_msgs::msg::Float64 override_msg;
          override_msg.data = 1.0;

          motion_override_pub_->publish(override_msg);

          response->success = true;
          response->message = "resume sent";
        });

    abort_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/firmware/abort",
        [this](
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
          bool ok = send_abort_to_firmware();

          response->success = ok;
          response->message = ok ? "abort sent" : "abort failed";
        });

    /* set_loc_mode_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/firmware/set_localization_anchored",
        [this](
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
          if (!last_state_)
          {
            response->success = false;
            response->message = "No firmware state yet";
            return;
          }

          if (!command_policy_.can_send(
                  WBP_CMD_SET_LOCALIZATION_MODE, *last_state_))
          {
            response->success = false;
            response->message = "Blocked by policy";
            return;
          }

          if (!command_writer_->is_connected())
          {
            response->success = false;
            response->message = "IPC not connected";
            return;
          }

          wbp_command_ipc_t cmd{};
          cmd.command = WBP_CMD_SET_LOCALIZATION_MODE;
          cmd.localization_mode =
              static_cast<uint8_t>(firmware::LocalizationMode::ANCHORED);

          command_writer_->send_raw(cmd);

          command_policy_.mark_sent(
              WBP_CMD_SET_LOCALIZATION_MODE, *last_state_);

          response->success = true;
          response->message = "Localization mode request sent (pending)";
        }); */

    clear_job_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/firmware/clear_job",
        [this](
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
          if (!last_state_)
          {
            response->success = false;
            response->message = "No firmware state yet";
            return;
          }

          if (!command_policy_.can_send(WBP_CMD_CLEAR_JOB, *last_state_))
          {
            response->success = false;
            response->message = "Blocked by policy";
            return;
          }

          if (!command_writer_->is_connected())
          {
            response->success = false;
            response->message = "IPC not connected";
            return;
          }

          if (!command_writer_->send_command(WBP_CMD_CLEAR_JOB))
          {
            response->success = false;
            response->message = "IPC write failed";
            return;
          }

          command_policy_.mark_sent(WBP_CMD_CLEAR_JOB, *last_state_);

          response->success = true;
          response->message = "clear_job sent (pending)";
        });

    /* ---------- Job Submit Service ---------- */
    job_submit_srv_ =
        this->create_service<wbp_interfaces::srv::JobSubmit>(
            "/firmware/job_submit",
            std::bind(
                &SupervisorNode::handle_job_submit,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    motion_state_sub_ =
        this->create_subscription<
            wbp_interfaces::msg::MotionRuntimeState>(
            "/motion_runtime_state",
            10,
            [this](const wbp_interfaces::msg::MotionRuntimeState::SharedPtr msg)
            {
              last_motion_state_ = *msg;

              // ================================
              // INDUSTRIAL AUTO-ABORT RULE (FIXED)
              // ================================
              if (msg->exec_state == 3) // FAULT
              {
                if (!abort_sent_)
                {
                  RCLCPP_ERROR(this->get_logger(),
                               "Motion layer fault detected → sending ABORT to firmware");

                  send_abort_to_firmware();
                  abort_sent_ = true;
                }
              }
              else
              {
                // Reset latch when recovered
                abort_sent_ = false;
              }
            });

    start_stream_pub_ =
        this->create_publisher<std_msgs::msg::Empty>(
            "/start_job_stream", 10);

    /* ---------- Timer ---------- */
    timer_ = this->create_wall_timer(
        50ms,
        std::bind(&SupervisorNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "Supervisor node started");
  }

private:
  // uint32_t last_sent_command_seq_{0};

  /* ============================================================
   * TIMER CALLBACK
   *  - Observes firmware state
   *  - Publishes MachineState
   *  - Evaluates command outcomes
   * ============================================================ */
  void on_timer()
  {
    if (!shm_state_)
    {
      return;
    }

    wbp_interfaces::msg::MachineState msg;
    msg.system_state = shm_state_->system_state;
    msg.safety_state = shm_state_->safety_state;
    msg.paused = shm_state_->paused;
    msg.job_id = shm_state_->job_id;
    msg.layer = shm_state_->layer;
    msg.segment = shm_state_->segment;

    msg.job_loaded = shm_state_->job_loaded;
    msg.loaded_job_id = shm_state_->loaded_job_id;
    msg.loaded_job_schema_version = shm_state_->loaded_job_schema_version;
    msg.loaded_job_hash = shm_state_->loaded_job_hash;
    msg.last_job_rejection = shm_state_->last_job_rejection;
    msg.last_command_seq = shm_state_->last_command_seq;
    msg.last_command_outcome = shm_state_->last_command_outcome;

    /* RCLCPP_INFO_THROTTLE(
        get_logger(),
        *this->get_clock(),
        1000,
        "[SUP STATE] job_id=%lu job_loaded=%d system=%d",
        msg.job_id,
        msg.job_loaded,
        msg.system_state); */

    /* ---------- Command outcome evaluation ---------- */
    if (last_state_)
    {
      ws::CommandOutcome outcome = command_policy_.evaluate(msg);

      switch (outcome)
      {
      case ws::CommandOutcome::ACCEPTED:
        RCLCPP_INFO(get_logger(), "Command ACCEPTED");
        break;

      case ws::CommandOutcome::REJECTED:
        RCLCPP_WARN(get_logger(), "Command REJECTED");
        break;

      case ws::CommandOutcome::FAULTED:
        RCLCPP_ERROR(get_logger(), "Command FAULTED");
        break;

      case ws::CommandOutcome::UNRESOLVED:
        /* no-op */
        break;
      }
    }

    last_state_ = msg;
    state_pub_->publish(msg);

    // 🔥 RESET START LATCH AFTER COMPLETION
    if (last_state_ &&
        last_state_->system_state == 2) // READY
    {
      start_sent_ = false;
    }

    // ================================
    // PAUSE → MOTION OVERRIDE CONTROL
    // ================================

    if (motion_override_pub_)
    {
      std_msgs::msg::Float64 override_msg;

      if (msg.paused)
      {
        override_msg.data = 0.0; // pause motion
      }
      else
      {
        override_msg.data = 1.0; // normal speed
      }

      motion_override_pub_->publish(override_msg);
    }

    // =======================================
    // ASYNC START TRIGGER (NEW LOGIC)
    // =======================================

    if (start_requested_ &&
        last_state_ &&
        last_state_->job_loaded && // 🔥 CRITICAL
        !start_sent_)
    {
      RCLCPP_INFO(get_logger(), "Sending START to firmware");

      send_command(WBP_CMD_START);

      start_sent_ = true;
      start_requested_ = false;
    }
  }

  /* ============================================================
   * JOB SUBMIT SERVICE
   *  - Phase 11.C1 only
   *  - Job IPC not implemented yet
   * ============================================================ */
  void
  handle_job_submit(
      const std::shared_ptr<wbp_interfaces::srv::JobSubmit::Request> request,
      std::shared_ptr<wbp_interfaces::srv::JobSubmit::Response> response)
  {
    std::cerr << "[SUP] received blob.size="
              << request->job_blob.size()
              << std::endl;

    bool ok = job_writer_->write_job(
        request->job_blob,
        request->source);

    response->accepted = ok;
    response->job_id = 0; // firmware is authoritative
    response->rejection_reason = ok ? "" : "Job IPC write failed";

    if (ok)
    {
      RCLCPP_INFO(get_logger(), "Job accepted. Waiting for START command.");
    }
  }

public:
  /* ============================================================
   * EXPLICIT COMMAND ENTRY POINT
   * ============================================================ */
  void send_command(wbp_command_type_t cmd)
  {
    if (!last_state_)
    {
      RCLCPP_WARN(get_logger(), "No MachineState yet; command ignored");
      return;
    }

    if (!command_policy_.can_send(cmd, *last_state_))
    {
      RCLCPP_WARN(get_logger(), "Command blocked by policy");
      return;
    }

    if (!command_writer_->is_connected())
    {
      RCLCPP_WARN(get_logger(), "Command IPC not connected yet");
      return;
    }

    if (!command_writer_->send_command(cmd))
    {
      RCLCPP_ERROR(get_logger(), "Failed to write command to IPC");
      return;
    }

    RCLCPP_INFO(get_logger(),
                "Sending command %d | system=%d job_id=%d",
                cmd,
                last_state_->system_state,
                last_state_->job_id);

    command_policy_.mark_sent(cmd, *last_state_);
    RCLCPP_INFO(get_logger(), "Command sent");
  }

  bool send_abort_to_firmware()
  {
    if (!command_writer_ || !command_writer_->is_connected())
      return false;

    if (!command_writer_->send_command(WBP_CMD_ABORT))
    {
      RCLCPP_WARN(get_logger(), "Failed to send ABORT to firmware");
      return false;
    }

    RCLCPP_ERROR(get_logger(), "ABORT sent to firmware");

    return true;
  }

  bool send_pause_to_firmware()
  {
    if (!last_state_)
    {
      RCLCPP_WARN(get_logger(), "No MachineState yet; pause ignored");
      return false;
    }

    if (!command_policy_.can_send(WBP_CMD_PAUSE, *last_state_))
    {
      RCLCPP_WARN(get_logger(), "Pause blocked by policy");
      return false;
    }

    if (!command_writer_->is_connected())
    {
      RCLCPP_WARN(get_logger(), "Command IPC not connected yet");
      return false;
    }

    if (!command_writer_->send_command(WBP_CMD_PAUSE))
    {
      RCLCPP_ERROR(get_logger(), "Failed to write PAUSE to IPC");
      return false;
    }

    command_policy_.mark_sent(WBP_CMD_PAUSE, *last_state_);

    std_msgs::msg::Float64 override_msg;
    override_msg.data = 0.0;
    motion_override_pub_->publish(override_msg);
    RCLCPP_INFO(get_logger(), "PAUSE command sent");

    return true;
  }

private:
  /* ---------- IPC ---------- */
  std::shared_ptr<IpcReader> ipc_reader_;
  wbp_machine_state_ipc_t *shm_state_{nullptr};
  std::unique_ptr<ws::IPCCommandWriter> command_writer_;
  std::unique_ptr<IPCJobWriter> job_writer_;

  /* ---------- Supervisor policy ---------- */
  ws::CommandPolicy command_policy_;
  std::optional<wbp_interfaces::msg::MachineState> last_state_;

  /* ---------- ROS ---------- */
  rclcpp::Publisher<wbp_interfaces::msg::MachineState>::SharedPtr state_pub_;
  rclcpp::Service<wbp_interfaces::srv::JobSubmit>::SharedPtr job_submit_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_job_srv_;

  rclcpp::Service<wbp_interfaces::srv::StartJob>::SharedPtr start_job_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<wbp_interfaces::msg::MotionRuntimeState>::SharedPtr motion_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_stream_pub_;
  std::optional<wbp_interfaces::msg::MotionRuntimeState> last_motion_state_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motion_override_pub_;

  bool abort_sent_{false};

  bool start_requested_{false};
  bool start_sent_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SupervisorNode>());
  rclcpp::shutdown();
  return 0;
}

// end of file: ros2_ws/src/wbp_supervisor/src/supervisor_node.cpp
