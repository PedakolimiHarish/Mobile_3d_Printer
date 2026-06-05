// start of file: ros2_ws/src/wbp_mobile_fabrication/src/world_motion_coordinator.cpp

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "wbp_interfaces/msg/base_motion_command.hpp"
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
// #include "wbp_interfaces/msg/base_motion_result.hpp"

#include <geometry_msgs/msg/twist.hpp>

class WorldMotionCoordinator : public rclcpp::Node
{
public:
    WorldMotionCoordinator()
        : Node("world_motion_coordinator")
    {
        tool_pose_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/tool_world_pose",
                10,
                std::bind(
                    &WorldMotionCoordinator::tool_pose_callback,
                    this,
                    std::placeholders::_1));

        target_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/world_tool_target",
                10,
                std::bind(
                    &WorldMotionCoordinator::target_callback,
                    this,
                    std::placeholders::_1));

        error_pub_ =
            create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/world_tool_error",
                10);

        /* cmd_vel_pub_ =
            create_publisher<
                geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10);

        odom_sub_ =
            create_subscription<
                nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(
                    &WorldMotionCoordinator::odom_callback,
                    this,
                    std::placeholders::_1)); */

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &WorldMotionCoordinator::update,
                    this));

        RCLCPP_INFO(
            get_logger(),
            "WorldMotionCoordinator started");
    }

private:
    void tool_pose_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_tool_pose_ = *msg;

        tool_pose_received_ = true;
    }

    void target_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_pose_ = *msg;

        target_received_ = true;
    }

    /* void odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        base_x_ =
            msg->pose.pose.position.x;
    } */

    void update()
    {

        if (!tool_pose_received_)
        {
            return;
        }

        if (!target_received_)
        {
            return;
        }

        geometry_msgs::msg::PoseStamped error;

        error.header.stamp = now();

        error.header.frame_id = "odom";

        error.pose.position.x =
            target_pose_.pose.position.x -
            current_tool_pose_.pose.position.x;

        error.pose.position.y =
            target_pose_.pose.position.y -
            current_tool_pose_.pose.position.y;

        error.pose.position.z =
            target_pose_.pose.position.z -
            current_tool_pose_.pose.position.z;

        error.pose.orientation.w = 1.0;

        error_pub_->publish(error);

        /* double error_x =
            error.pose.position.x;

        double error_y =
            error.pose.position.y;

        geometry_msgs::msg::Twist cmd;


        cmd.linear.x =
            std::clamp(
                error_x * 0.5,
                -0.20,
                0.20);



        cmd.angular.z =
            std::clamp(
                error_y * 0.5,
                -0.25,
                0.25);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "error=(%.3f %.3f %.3f) cmd=(%.3f %.3f)",
            error.pose.position.x,
            error.pose.position.y,
            error.pose.position.z,
            cmd.linear.x,
            cmd.angular.z);

        cmd_vel_pub_->publish(cmd); */
    }

private:
    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr tool_pose_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr error_pub_;

    /* rclcpp::Publisher<
        geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_pub_;

    rclcpp::Subscription<
        nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr
        error_sub_;

    double base_x_ = 0.0; */

    geometry_msgs::msg::PoseStamped current_tool_pose_;

    geometry_msgs::msg::PoseStamped target_pose_;

    bool tool_pose_received_ = false;

    bool target_received_ = false;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<
            WorldMotionCoordinator>());

    rclcpp::shutdown();

    return 0;
}

// end of file: ros2_ws/src/wbp_mobile_fabrication/src/world_motion_coordinator.cpp