// start of file: ros2_ws/src/wbp_firmware_motion_executor/src/motion_monitor.cpp
#include "wbp_firmware_motion_executor/motion_monitor.hpp"

#include <sstream>
#include <iomanip>

MotionMonitor::MotionMonitor()
    : Node("motion_monitor")
{
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        10,
        std::bind(&MotionMonitor::jointCallback, this, std::placeholders::_1));

    status_sub_ = this->create_subscription<wbp_interfaces::msg::SegmentStatus>(
        "/segment_status",
        10,
        std::bind(&MotionMonitor::statusCallback, this, std::placeholders::_1));

    // 10 Hz print rate
    print_timer_ = this->create_timer(
        std::chrono::milliseconds(100),
        std::bind(&MotionMonitor::printLoop, this));

    runtime_sub_ =
        this->create_subscription<
            wbp_interfaces::msg::MotionRuntimeState>(
            "/motion_runtime_state",
            10,
            std::bind(
                &MotionMonitor::runtimeCallback,
                this,
                std::placeholders::_1));
}

void MotionMonitor::jointCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    last_joint_state_ = *msg;
}

void MotionMonitor::statusCallback(
    const wbp_interfaces::msg::SegmentStatus::SharedPtr msg)
{
    // 1 = executing
    if (msg->status == 1)
    {
        segment_active_ = true;
        current_segment_id_ = msg->segment_id;

        RCLCPP_INFO(this->get_logger(),
                    "---- SEGMENT %u START ----",
                    current_segment_id_);
    }

    // 0 = queued, 4 = fault, etc
    if (msg->status != 1)
    {
        if (segment_active_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "---- SEGMENT %u END ----",
                        current_segment_id_);
        }

        segment_active_ = false;
    }
}

void MotionMonitor::printLoop()
{
    if (!segment_active_)
        return;

    if (last_joint_state_.name.empty())
        return;

    std::stringstream ss;

    ss << "SEG " << current_segment_id_ << " | ";

    ss << "SEG " << current_segment_id_
       << " | EXEC=" << (int)exec_state_
       << " | QUEUE=" << queue_depth_
       << " | ";

    for (size_t i = 0; i < last_joint_state_.name.size(); ++i)
    {
        ss << last_joint_state_.name[i]
           << ": pos=" << std::fixed << std::setprecision(3)
           << last_joint_state_.position[i];

        if (i < last_joint_state_.velocity.size())
        {
            ss << " vel=" << std::fixed << std::setprecision(3)
               << last_joint_state_.velocity[i];
        }

        ss << " | ";
    }

    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}

void MotionMonitor::runtimeCallback(
    const wbp_interfaces::msg::MotionRuntimeState::SharedPtr msg)
{
    exec_state_ = msg->exec_state;
    queue_depth_ = msg->queue_depth;
}

// end of file: ros2_ws/src/wbp_firmware_motion_executor/src/motion_monitor.cpp