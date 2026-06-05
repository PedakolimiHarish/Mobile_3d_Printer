// start of file: firmware/src/motion/ros_sim_motion_controller.cpp
#include "firmware/motion/ros_sim_motion_controller.hpp"
#include "wbp_sim_interfaces/msg/motion_result.hpp"
#include "wbp_sim_interfaces/msg/motion_target.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <std_msgs/msg/u_int32.hpp>

#include "wbp_sim_interfaces/msg/motion_command.hpp"
#include "wbp_sim_interfaces/msg/motion_result.hpp"
#include "firmware/machine_state.hpp"

#include <std_msgs/msg/empty.hpp>

namespace firmware
{
  MachineState &get_machine_state();

  using MotionCommandMsg = wbp_sim_interfaces::msg::MotionCommand;
  using MotionResultMsg = wbp_sim_interfaces::msg::MotionResult;

  RosSimMotionController::RosSimMotionController()
  {
    // Initialize ROS once
    static bool ros_initialized = false;
    if (!ros_initialized)
    {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
      ros_initialized = true;
    }

    ros_node_ = std::make_shared<rclcpp::Node>("firmware_ros_sim_motion");

    // Create publisher (STORE AS MEMBER)
    command_pub_ =
        ros_node_->create_publisher<MotionCommandMsg>(
            "/sim/motion/command", 10);

    // Create subscriber
    result_sub_ = ros_node_->create_subscription<MotionResultMsg>(
        "/sim/motion/result",
        10,
        [this](const MotionResultMsg::SharedPtr msg)
        {
          this->on_motion_result(msg->sequence_id, msg->status);
        });

    status_sub_ =
        ros_node_->create_subscription<wbp_sim_interfaces::msg::MotionStatus>(
            "/sim/motion/status",
            10,
            [this](const wbp_sim_interfaces::msg::MotionStatus::SharedPtr msg)
            {
              bool was_ready = executor_ready_.load();

              executor_ready_.store(msg->ready);

              if (!was_ready && msg->ready)
              {
                std::cerr << "[FW MOTION] executor READY\n";
              }

              on_motion_status(
                  msg->executing_segment,
                  msg->queue_depth,
                  msg->queue_capacity);
            });

    target_pub_ = ros_node_->create_publisher<
        wbp_sim_interfaces::msg::MotionTarget>(
        "/sim/motion/target", 10);

    world_complete_sub_ =
        ros_node_->create_subscription<std_msgs::msg::UInt32>(
            "/firmware/world_motion_complete",
            10,
            [this](const std_msgs::msg::UInt32::SharedPtr msg)
            {
              on_world_complete(msg->data);
            });

    abort_pub_ = ros_node_->create_publisher<std_msgs::msg::Empty>(
        "/motion_abort", 10);

    // Spin ROS in dedicated thread
    std::thread([node = ros_node_]()
                { rclcpp::spin(node); })
        .detach();

    executor_ready_.store(true);
  }

  RosSimMotionController::~RosSimMotionController()
  {
    // Intentionally empty
  }

  uint32_t RosSimMotionController::inflight() const
  {
    return last_submitted_seq_.load() - last_completed_seq_.load();
  }

  bool RosSimMotionController::is_ready() const
  {

    if (!executor_ready_.load())
    {
      return false;
    }
    uint32_t cap = executor_queue_capacity_.load();
    uint32_t depth = executor_queue_depth_.load();

    if (cap == 0)
      return inflight() < STREAM_WINDOW;

    uint32_t target = cap > 2 ? cap - 2 : cap;

    if (depth >= target)
      return false;

    if (inflight() >= target)
      return false;

    return true;
  }

  MotionResult RosSimMotionController::submit(const MotionCommand &cmd)
  {

    if (!is_ready())
    {
      std::cerr << "[FW MOTION] submit rejected — busy\n";
      return MotionResult::REJECTED;
    }

    uint32_t seq = ++last_submitted_seq_;

    publish_motion_command(seq, cmd);

    std::cerr << "[FW MOTION] submit seq=" << seq
              << " inflight=" << inflight()
              << std::endl;

    return MotionResult::ACCEPTED;
  }

  void RosSimMotionController::publish_motion_command(
      uint32_t seq, const MotionCommand &cmd)
  {
    // 🔴 ADD THIS DEBUG HERE
    std::cerr << "[FW DEBUG] raw cmd: seq=" << seq
              << " x=" << cmd.x
              << " y=" << cmd.y
              << " z=" << cmd.z
              << " F=" << cmd.feedrate
              << " E=" << cmd.extrusion
              << std::endl;

    // 1️⃣ existing intent message
    MotionCommandMsg msg;
    msg.sequence_id = seq;
    msg.intent = static_cast<uint8_t>(cmd.intent);
    command_pub_->publish(msg);

    // 2️⃣ geometry message
    wbp_sim_interfaces::msg::MotionTarget target;
    target.sequence_id = seq;
    target.x = cmd.x;
    target.y = cmd.y;
    target.z = cmd.z;
    target.feedrate = cmd.feedrate;
    target.extrusion = cmd.extrusion;

    target_pub_->publish(target);
  }

  MotionResult RosSimMotionController::poll()
  {

    auto &ms = get_machine_state(); // 🔥 you already use this pattern elsewhere

    bool idle = (inflight() == 0);

    // 🔥 ALSO treat FAULT as idle (pipeline is dead)
    if (abort_active_ && last_completed_seq_.load() < last_submitted_seq_.load())
    {
      // If ROS stopped responding → assume forced stop
      uint64_t now =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count();

      uint64_t last_activity = std::max(
          last_status_time_.load(),
          last_result_time_.load());

      if (last_activity > 0 && (now - last_activity) > 500)
      {
        std::cerr << "[FW MOTION] Abort fallback → treating as drained\n";
        idle = true;
      }
    }

    ms.motion_idle = idle;

    if (check_watchdog())
    {
      return MotionResult::FAULTED;
    }

    static uint32_t last_reported = 0;

    uint32_t completed = last_completed_seq_.load();

    if (completed > last_reported)
    {
      last_reported = completed;

      std::cerr << "[FW MOTION] completed seq=" << completed << std::endl;

      return MotionResult::COMPLETED;
    }

    // 🔥 ABORT DRAIN DETECTION
    if (abort_active_ && inflight() == 0)
    {
      std::cerr << "[FW MOTION] Abort fully drained\n";

      abort_active_.store(false); // 🔥 CLEAR
    }

    return MotionResult::IN_PROGRESS;
  }

  void RosSimMotionController::emergency_stop()
  {
    std::cerr << "[FW MOTION] EMERGENCY STOP\n";
  }

  void RosSimMotionController::on_motion_result(
      uint32_t seq,
      uint8_t status)
  {
    std::cerr << "[FW MOTION] result seq="
              << seq
              << " status="
              << int(status)
              << std::endl;

    last_result_time_.store(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());

    // 🔥 Phase 12 — release slot ONCE at IN_PROGRESS
    /* if (status == MotionResultMsg::IN_PROGRESS)
    {
      if (!slot_released_[seq])
      {
        last_completed_seq_.store(seq);
        slot_released_[seq] = true;
      }
    } */

    if (status == MotionResultMsg::IN_PROGRESS)
    {
      slot_released_[seq] = true;
    }

    // cleanup after completion
    if (status == MotionResultMsg::COMPLETED)
    {
      slot_released_.erase(seq);

      std::cerr << "[FW MOTION] completed seq=" << seq << std::endl;
    }
  }

  bool RosSimMotionController::check_watchdog()
  {
    // 🔒 Do NOT watchdog during abort or pause
    if (abort_active_)
    {
      return false;
    }

    uint64_t now =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();

    uint64_t last_status = last_status_time_.load();
    uint64_t last_result = last_result_time_.load();

    uint64_t last_activity = std::max(last_status, last_result);

    if (last_activity == 0)
      return false;

    // 🔒 ONLY watchdog if work is actually in-flight
    if (inflight() > 0 && (now - last_activity > WATCHDOG_TIMEOUT_MS))
    {
      std::cerr << "[FW MOTION] PIPELINE WATCHDOG TIMEOUT\n";
      return true;
    }

    return false;
  }

  void RosSimMotionController::abort()
  {
    if (!abort_pub_)
      return;

    abort_active_.store(true); // 🔥 ADD THIS

    std_msgs::msg::Empty msg;
    abort_pub_->publish(msg);

    std::cerr << "[FW MOTION] ABORT sent to ROS\n";
  }

  bool RosSimMotionController::is_idle() const
  {
    return inflight() == 0;
  }

  void RosSimMotionController::set_abort_active(bool active)
  {
    abort_active_.store(active);

    if (active)
    {
      std::cerr << "[FW MOTION] abort_active = true\n";
    }
  }

  void RosSimMotionController::on_motion_status(
      uint32_t executing,
      uint32_t depth,
      uint32_t capacity)
  {
    executor_queue_depth_.store(depth);
    executor_queue_capacity_.store(capacity);

    last_status_time_.store(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());

    std::cerr << "[FW MOTION] status exec="
              << executing
              << " depth=" << depth
              << " cap=" << capacity
              << std::endl;
  }

  void RosSimMotionController::on_world_complete(uint32_t seq)
  {
    std::cerr
        << "[FW MOTION] world complete seq="
        << seq
        << std::endl;

    last_completed_seq_.store(seq);

    std::cerr
        << "[FW MOTION] inflight="
        << inflight()
        << std::endl;

    last_result_time_.store(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
  }

} // namespace firmware

// end of file: firmware/src/motion/ros_sim_motion_controller.cpp