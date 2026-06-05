// start of file: firmware/include/firmware/motion/ros_sim_motion_controller.hpp
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include "wbp_sim_interfaces/msg/motion_command.hpp"
#include "firmware/motion/motion_controller.hpp"
#include "wbp_sim_interfaces/msg/motion_result.hpp"
#include "wbp_sim_interfaces/msg/motion_status.hpp"
#include "wbp_sim_interfaces/msg/motion_target.hpp"

#include <std_msgs/msg/empty.hpp>
namespace firmware
{

  class RosSimMotionController final : public MotionController
  {
  public:
    RosSimMotionController();
    ~RosSimMotionController() override;

    MotionResult submit(const MotionCommand &cmd) override;
    MotionResult poll() override;

    void emergency_stop() override;
    bool is_ready() const override;
    void abort() override;
    uint32_t inflight() const;
    void set_abort_active(bool active) override;

  private:
    void publish_motion_command(uint32_t seq, const MotionCommand &cmd);
    void on_motion_result(uint32_t seq, uint8_t status);
    void on_motion_status(uint32_t executing, uint32_t depth, uint32_t capacity);
    bool is_idle() const override;
    bool check_watchdog();
    void on_world_complete(uint32_t seq);

    std::shared_ptr<rclcpp::Node> ros_node_;
    std::atomic<bool> executor_ready_{false};
    std::unordered_map<uint32_t, bool> slot_released_;
    rclcpp::Publisher<wbp_sim_interfaces::msg::MotionCommand>::SharedPtr command_pub_;
    rclcpp::Subscription<wbp_sim_interfaces::msg::MotionResult>::SharedPtr result_sub_;
    rclcpp::Subscription<wbp_sim_interfaces::msg::MotionStatus>::SharedPtr status_sub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr abort_pub_;
    rclcpp::Publisher<wbp_sim_interfaces::msg::MotionTarget>::SharedPtr target_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr world_complete_sub_;

    std::atomic<uint32_t> executor_queue_depth_{0};
    std::atomic<uint32_t> executor_queue_capacity_{0};

    std::atomic<bool> abort_active_{false};

    // Streaming state
    std::atomic<uint32_t> last_submitted_seq_{0};
    std::atomic<uint32_t> last_completed_seq_{0};

    std::atomic<uint64_t> last_status_time_{0};
    std::atomic<uint64_t> last_result_time_{0};

    static constexpr uint32_t STREAM_WINDOW = 5;

    static constexpr uint64_t WATCHDOG_TIMEOUT_MS = 5000;
  };

}
