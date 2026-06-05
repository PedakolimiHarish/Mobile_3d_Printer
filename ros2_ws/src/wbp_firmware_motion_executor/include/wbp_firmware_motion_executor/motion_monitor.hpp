#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wbp_interfaces/msg/segment_status.hpp>
#include <wbp_interfaces/msg/motion_runtime_state.hpp>

class MotionMonitor : public rclcpp::Node
{
public:
    MotionMonitor();

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<wbp_interfaces::msg::SegmentStatus>::SharedPtr status_sub_;
    rclcpp::TimerBase::SharedPtr print_timer_;

    bool segment_active_ = false;
    uint32_t current_segment_id_ = 0;
    uint8_t exec_state_ = 0;
    size_t queue_depth_ = 0;

    rclcpp::Subscription<wbp_interfaces::msg::MotionRuntimeState>::SharedPtr runtime_sub_;

    sensor_msgs::msg::JointState last_joint_state_;

    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void statusCallback(const wbp_interfaces::msg::SegmentStatus::SharedPtr msg);
    void runtimeCallback(const wbp_interfaces::msg::MotionRuntimeState::SharedPtr msg);
    void printLoop();
};