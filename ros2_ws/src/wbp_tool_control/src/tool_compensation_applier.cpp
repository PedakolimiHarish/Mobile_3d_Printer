// start of file: ros2_ws/src/wbp_tool_control/src/tool_compensation_applier.cpp
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "wbp_interfaces/msg/tool_compensation.hpp"

class ToolCompensationApplier : public rclcpp::Node
{
public:
    ToolCompensationApplier()
        : Node("tool_compensation_applier")
    {
        compensation_sub_ =
            create_subscription<
                wbp_interfaces::msg::ToolCompensation>(
                "/tool_compensation_offset",
                10,
                std::bind(
                    &ToolCompensationApplier::compensation_callback,
                    this,
                    std::placeholders::_1));

        tool_target_pub_ =
            create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/tool_target_pose",
                10);

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &ToolCompensationApplier::publish_target,
                    this));

        RCLCPP_INFO(
            get_logger(),
            "ToolCompensationApplier started");
    }

private:
    void compensation_callback(
        const wbp_interfaces::msg::ToolCompensation::SharedPtr msg)
    {
        if (!msg->valid)
        {
            return;
        }

        /* geometry_msgs::msg::PoseStamped target;

        target.header.stamp = now();

        target.header.frame_id = "tool0"; */

        constexpr double alpha = 0.1;

        if (std::abs(msg->dx) > 0.0001)
        {
            target_dx_ = msg->dx;
        }

        if (std::abs(msg->dy) > 0.0001)
        {
            target_dy_ = msg->dy;
        }

        if (std::abs(msg->dz) > 0.0001)
        {
            target_dz_ = msg->dz;
        }

        current_dx_ =
            current_dx_ +
            alpha * (target_dx_ - current_dx_);

        current_dy_ =
            current_dy_ +
            alpha * (target_dy_ - current_dy_);

        current_dz_ =
            current_dz_ +
            alpha * (target_dz_ - current_dz_);

        constexpr double max_x = 3.0;
        constexpr double max_y = 3.0;
        constexpr double max_z = 1.0;

        current_dx_ =
            std::clamp(current_dx_, -max_x, max_x);

        current_dy_ =
            std::clamp(current_dy_, -max_y, max_y);

        current_dz_ =
            std::clamp(current_dz_, -max_z, max_z);

        constexpr double deadband = 0.01;

        if (std::abs(current_dx_) < deadband)
        {
            current_dx_ = 0.0;
        }

        if (std::abs(current_dy_) < deadband)
        {
            current_dy_ = 0.0;
        }

        if (std::abs(current_dz_) < deadband)
        {
            current_dz_ = 0.0;
        }
    }

    void publish_target()
    {
        geometry_msgs::msg::PoseStamped target;

        target.header.stamp = now();

        target.header.frame_id = "tool0";

        target.pose.position.x = current_dx_;
        target.pose.position.y = current_dy_;
        target.pose.position.z = current_dz_;

        target.pose.orientation.w = 1.0;

        tool_target_pub_->publish(target);
    }

private:
    rclcpp::Subscription<
        wbp_interfaces::msg::ToolCompensation>::SharedPtr compensation_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr tool_target_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    double current_dx_ = 0.0;
    double current_dy_ = 0.0;
    double current_dz_ = 0.0;

    double target_dx_ = 0.0;
    double target_dy_ = 0.0;
    double target_dz_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<ToolCompensationApplier>());

    rclcpp::shutdown();

    return 0;
}

// end of file: ros2_ws/src/wbp_tool_control/src/tool_compensation_applier.cpp