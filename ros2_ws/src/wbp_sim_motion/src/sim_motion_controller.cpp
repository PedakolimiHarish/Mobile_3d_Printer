// start of file: ros2_ws/src/wbp_sim_motion/src/sim_motion_controller.cpp
#include <rclcpp/rclcpp.hpp>
#include "wbp_sim_interfaces/msg/motion_command.hpp"
#include "wbp_sim_interfaces/msg/motion_result.hpp"
#include "wbp_sim_interfaces/msg/motion_target.hpp"
#include <unordered_map>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <queue>

// std::queue<uint32_t> queue_;
using MotionCommand = wbp_sim_interfaces::msg::MotionCommand;
using MotionResult = wbp_sim_interfaces::msg::MotionResult;

class SimMotionController : public rclcpp::Node
{
public:
  SimMotionController()
      : Node("sim_motion_controller")
  {

    command_sub_ = this->create_subscription<MotionCommand>(
        "/sim/motion/command",
        10,
        std::bind(&SimMotionController::on_command, this, std::placeholders::_1));

    result_pub_ = this->create_publisher<MotionResult>(
        "/sim/motion/result",
        10);

    target_sub_ = this->create_subscription<
        wbp_sim_interfaces::msg::MotionTarget>(
        "/sim/motion/target",
        10,
        [this](const wbp_sim_interfaces::msg::MotionTarget::SharedPtr msg)
        {
          targets_[msg->sequence_id] = {
              {msg->x, msg->y, msg->z, msg->extrusion},
              msg->feedrate};
        });

    joint_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arm_controller/commands",
        10);

    tick_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&SimMotionController::on_tick, this));

    RCLCPP_INFO(this->get_logger(), "SimMotionController started");
  }

private:
  std::vector<double> current_pos_ = {0.0, 0.0, 0.0, 0.0};
  std::vector<double> target_ = {0.0, 0.0, 0.0, 0.0};
  double current_feedrate_ = 1.0;
  uint32_t active_seq_ = 0;
  bool moving_ = false;
  rclcpp::Time segment_start_time_;
  double segment_duration_ = 0.0;

  std::queue<uint32_t> command_queue_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;

  static constexpr int TICKS_PER_COMMAND = 3;

  std::vector<double> start_pos_ = {0.0, 0.0, 0.0, 0.0};
  double segment_length_ = 0.0;

  struct Target
  {
    std::vector<double> pos; // x,y,z,e
    double feedrate;
  };

  std::unordered_map<uint32_t, Target> targets_;

  rclcpp::Subscription<MotionCommand>::SharedPtr command_sub_;
  rclcpp::Publisher<MotionResult>::SharedPtr result_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::Subscription<wbp_sim_interfaces::msg::MotionTarget>::SharedPtr target_sub_;

  void on_command(const MotionCommand::SharedPtr msg)
  {
    command_queue_.push(msg->sequence_id);

    publish_result(msg->sequence_id, MotionResult::ACCEPTED);

    RCLCPP_INFO(this->get_logger(),
                "Queued command seq=%u", msg->sequence_id);
  }

  void on_tick()
  {
    // ===============================
    // START NEW SEGMENT
    // ===============================
    if (!moving_ && !command_queue_.empty())
    {
      uint32_t seq = command_queue_.front();

      auto it = targets_.find(seq);
      if (it == targets_.end())
        return;

      command_queue_.pop();

      // target
      target_ = it->second.pos;
      target_[3] = 0.0; // extruder disabled

      start_pos_ = current_pos_;

      // 1️⃣ compute segment length FIRST
      segment_length_ = 0.0;
      for (int i = 0; i < 3; i++)
      {
        double d = target_[i] - start_pos_[i];
        segment_length_ += d * d;
      }
      segment_length_ = std::sqrt(segment_length_);

      // 🔥 ZERO-LENGTH SEGMENT HANDLING
      if (segment_length_ < 1e-6)
      {
        RCLCPP_WARN(this->get_logger(),
                    "Zero-length segment seq=%u → instant complete",
                    seq);

        // Snap position (already same, but safe)
        current_pos_ = target_;

        publish_result(seq, MotionResult::COMPLETED);

        // DO NOT enter moving state
        moving_ = false;

        return;
      }

      // 2️⃣ set feedrate
      current_feedrate_ = it->second.feedrate;

      // 🔥 FEEDRATE SAFETY CLAMP
      if (current_feedrate_ < 1e-6)
      {
        RCLCPP_WARN(this->get_logger(),
                    "Feedrate too small → clamped");

        current_feedrate_ = 1e-6;
      }

      // 3️⃣ compute velocity (SIM scaling)
      double velocity = current_feedrate_ * 200.0;

      // 4️⃣ compute duration
      if (velocity > 1e-6)
        segment_duration_ = segment_length_ / velocity;
      else
        segment_duration_ = 0.0;

      // 🔥 MINIMUM DURATION (avoid instant jump)
      if (segment_duration_ < 0.02)
      {
        segment_duration_ = 0.02; // 1 tick minimum
      }

      // 5️⃣ set start time LAST
      segment_start_time_ = this->now();

      active_seq_ = seq;
      moving_ = true;

      publish_result(active_seq_, MotionResult::IN_PROGRESS);

      RCLCPP_INFO(this->get_logger(),
                  "Start seq=%u | len=%.4f | F=%.6f | dur=%.4f",
                  seq, segment_length_, current_feedrate_, segment_duration_);
    }

    if (!moving_)
      return;

    // ===============================
    // TIME-BASED INTERPOLATION
    // ===============================
    double elapsed =
        (this->now() - segment_start_time_).seconds();

    double progress = 0.0;

    if (segment_duration_ > 1e-6)
      progress = elapsed / segment_duration_;

    if (progress > 1.0)
      progress = 1.0;

    // interpolate XYZ
    for (int i = 0; i < 3; i++)
    {
      current_pos_[i] =
          start_pos_[i] +
          progress * (target_[i] - start_pos_[i]);
    }

    // extruder disabled
    current_pos_[3] = 0.0;

    // publish joint command
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = {
        current_pos_[0],
        current_pos_[1],
        current_pos_[2],
        0.0};

    joint_cmd_pub_->publish(cmd);

    // ===============================
    // COMPLETION
    // ===============================
    if (progress >= 1.0)
    {
      // 🔥 FORCE EXACT TARGET (no drift)
      current_pos_ = target_;

      moving_ = false;

      publish_result(active_seq_, MotionResult::COMPLETED);

      RCLCPP_INFO(this->get_logger(),
                  "Completed seq=%u", active_seq_);
    }
  }

  void publish_result(uint32_t seq, uint8_t status)
  {
    MotionResult res;
    res.sequence_id = seq;
    res.status = status;
    result_pub_->publish(res);
  }
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimMotionController>());
  rclcpp::shutdown();
  return 0;
}

// end of file: ros2_ws/src/wbp_sim_motion/src/sim_motion_controller.cpp
