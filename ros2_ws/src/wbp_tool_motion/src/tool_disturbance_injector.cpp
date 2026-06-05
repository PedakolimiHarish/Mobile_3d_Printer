// start of file: ros2_ws/src/wbp_tool_motion/src/tool_disturbance_injector.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cmath>

class ToolDisturbanceInjector : public rclcpp::Node
{
public:
    ToolDisturbanceInjector()
        : Node("tool_disturbance_injector")
    {
        disturbed_pub_ =
            create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/tool_disturbance",
                10);

        odom_sub_ =
            create_subscription<
                nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(
                    &ToolDisturbanceInjector::odom_callback,
                    this,
                    std::placeholders::_1));

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &ToolDisturbanceInjector::publish_disturbance,
                    this));

        RCLCPP_INFO(
            get_logger(),
            "ToolDisturbanceInjector started");
    }

private:
    void publish_disturbance()
    {
        geometry_msgs::msg::PoseStamped msg;

        msg.header.stamp = now();

        msg.header.frame_id = "tool0";

        double linear_acceleration =
            linear_velocity_ -
            previous_linear_velocity_;

        double angular_acceleration =
            angular_velocity_ -
            previous_angular_velocity_;

        previous_linear_velocity_ =
            linear_velocity_;

        previous_angular_velocity_ =
            angular_velocity_;

        msg.pose.position.x =
            0.5 * linear_acceleration;

        msg.pose.position.y =
            0.2 * angular_acceleration;

        msg.pose.position.z = 0.0;

        msg.pose.orientation.w = 1.0;

        disturbed_pub_->publish(msg);
    }

    void odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        linear_velocity_ =
            msg->twist.twist.linear.x;

        angular_velocity_ =
            msg->twist.twist.angular.z;
    }

private:
    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr disturbed_pub_;

    rclcpp::Subscription<
        nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;

    double previous_linear_velocity_ = 0.0;
    double previous_angular_velocity_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<ToolDisturbanceInjector>());

    rclcpp::shutdown();

    return 0;
}

// end of file: ros2_ws/src/wbp_tool_motion/src/tool_disturbance_injector.cpp