// start of file: ros2_ws/src/wbp_tool_control/src/physical_compensation_controller.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "wbp_interfaces/msg/tool_compensation.hpp"
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "wbp_interfaces/msg/base_motion_result.hpp"
#include <chrono>
#include <cmath>

class PhysicalCompensationController
    : public rclcpp::Node
{
public:
    PhysicalCompensationController()
        : Node("physical_compensation_controller")
    {
        target_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/world_tool_target",
                10,
                std::bind(
                    &PhysicalCompensationController::
                        target_callback,
                    this,
                    std::placeholders::_1));

        base_result_sub_ =
            create_subscription<
                wbp_interfaces::msg::BaseMotionResult>(
                "/base/motion_result",
                10,
                std::bind(
                    &PhysicalCompensationController::
                        base_result_callback,
                    this,
                    std::placeholders::_1));

        error_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/world_tool_error",
                10,
                std::bind(
                    &PhysicalCompensationController::
                        error_callback,
                    this,
                    std::placeholders::_1));

        odom_sub_ =
            create_subscription<
                nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(
                    &PhysicalCompensationController::
                        odom_callback,
                    this,
                    std::placeholders::_1));

        arm_command_pub_ =
            create_publisher<
                std_msgs::msg::Float64MultiArray>(
                "/arm_controller/commands",
                10);

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(
                    &PhysicalCompensationController::update,
                    this));

        RCLCPP_INFO(
            get_logger(),
            "PhysicalCompensationController started");
    }

private:
    void odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        base_x_ =
            msg->pose.pose.position.x;

        base_y_ =
            msg->pose.pose.position.y;
    }

    void target_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_pose_ = *msg;
        target_received_ = true;
    }

    void error_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double error_x =
            msg->pose.position.x;

        double error_z =
            msg->pose.position.z;

        double error_y =
            msg->pose.position.y;

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "ERROR ex=%.3f ey=%.3f ez=%.3f",
            error_x,
            error_y,
            error_z);

        constexpr double ERROR_DEADBAND = 0.02;
        constexpr double ERROR_GAIN = 0.015;
        constexpr double MAX_CORRECTION = 0.05;

        double correction_x = 0.0;
        double correction_z = 0.0;

        if (std::abs(error_x) > ERROR_DEADBAND)
        {
            correction_x =
                std::clamp(
                    error_x * ERROR_GAIN,
                    -MAX_CORRECTION,
                    MAX_CORRECTION);
        }

        if (std::abs(error_z) > ERROR_DEADBAND)
        {
            correction_z =
                std::clamp(
                    error_z * ERROR_GAIN,
                    -MAX_CORRECTION,
                    MAX_CORRECTION);
        }

        current_x_ =
            desired_x_ +
            correction_x;

        current_z_ =
            desired_z_ +
            correction_z;

        current_x_ =
            std::clamp(
                current_x_,
                -1.0,
                1.0);

        current_z_ =
            std::clamp(
                current_z_,
                -1.0,
                1.0);

        std_msgs::msg::Float64MultiArray cmd;

        cmd.data =
            {
                current_theta_,
                current_z_,
                current_x_,
                0.0};

        arm_command_pub_->publish(cmd);
    }

    void base_result_callback(
        const wbp_interfaces::msg::
            BaseMotionResult::SharedPtr msg)
    {
        if (
            msg->status ==
            wbp_interfaces::msg::
                BaseMotionResult::IN_PROGRESS)
        {
            base_motion_active_ = true;
        }

        if (
            msg->status ==
            wbp_interfaces::msg::
                BaseMotionResult::COMPLETED)
        {
            base_motion_active_ = false;
        }
    }

    void update()
    {
        if (!target_received_)
            return;

        constexpr double arm_neutral_x = 1.90;
        constexpr double neutral_tool_z = 1.375;

        double target_x =
            target_pose_.pose.position.x;

        double target_y =
            target_pose_.pose.position.y;

        double target_z =
            target_pose_.pose.position.z;

        double dx =
            target_x - base_x_;

        double dy =
            target_y - base_y_;

        double radius =
            std::sqrt(dx * dx + dy * dy);

        double carriage_extension =
            radius - arm_neutral_x;

        desired_x_ =
            std::clamp(
                carriage_extension,
                -1.0,
                1.0);

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "ARM radius=%.3f ext=%.3f desired_x=%.3f base=(%.3f %.3f) target=(%.3f %.3f)",
            radius,
            carriage_extension,
            desired_x_,
            base_x_,
            base_y_,
            target_x,
            target_y);

        desired_z_ =
            std::clamp(
                target_z - neutral_tool_z,
                -1.0,
                1.0);

        current_theta_ =
            std::atan2(dy, dx);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "BASE=%.3f TARGET=%.3f RADIUS=%.3f EXT=%.3f",
            base_x_,
            target_x,
            radius,
            desired_x_);
    }

private:
    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr
        target_sub_;

    rclcpp::Publisher<
        std_msgs::msg::Float64MultiArray>::SharedPtr
        arm_command_pub_;

    rclcpp::Subscription<
        nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr
        error_sub_;

    geometry_msgs::msg::PoseStamped
        world_error_;

    rclcpp::Subscription<
        wbp_interfaces::msg::BaseMotionResult>::SharedPtr
        base_result_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool error_received_ = false;

    double current_x_ = 0.0;
    double prev_error_x_ = 0.0;

    double current_z_ = 0.0;
    constexpr static double gain_ = 1.2;
    double prev_error_z_ = 0.0;
    bool base_motion_active_ = false;

    double current_theta_ = 0.0;

    double prev_theta_error_ = 0.0;

    double base_x_ = 0.0;
    double base_y_ = 0.0;

    double desired_x_ = 0.0;
    double desired_z_ = 0.0;

    geometry_msgs::msg::PoseStamped target_pose_;
    bool target_received_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<
            PhysicalCompensationController>());
    rclcpp::shutdown();
    return 0;
}

// end of file: ros2_ws/src/wbp_tool_control/src/physical_compensation_controller.cpp