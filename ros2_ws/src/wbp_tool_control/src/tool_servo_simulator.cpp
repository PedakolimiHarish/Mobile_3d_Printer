// start of file: ros2_ws/src/wbp_tool_control/src/tool_servo_simulator.cpp
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cmath>

class ToolServoSimulator : public rclcpp::Node
{
public:
    ToolServoSimulator()
        : Node("tool_servo_simulator")
    {
        target_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/sim_tool_motion_command",
                10,
                std::bind(
                    &ToolServoSimulator::target_callback,
                    this,
                    std::placeholders::_1));

        disturbance_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/tool_disturbance",
                10,
                std::bind(
                    &ToolServoSimulator::disturbance_callback,
                    this,
                    std::placeholders::_1));

        current_pub_ =
            create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/tool_current_pose",
                10);

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &ToolServoSimulator::update_loop,
                    this));

        RCLCPP_INFO(
            get_logger(),
            "ToolServoSimulator started");
    }

private:
    void target_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_x_ = msg->pose.position.x;
        target_y_ = msg->pose.position.y;
        target_z_ = msg->pose.position.z;
    }

    void disturbance_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        disturbance_x_ =
            msg->pose.position.x;

        disturbance_y_ =
            msg->pose.position.y;

        disturbance_z_ =
            msg->pose.position.z;
    }

    void update_loop()
    {
        constexpr double gain = 0.1;

        current_x_ +=
            gain * (target_x_ - current_x_);

        current_y_ +=
            gain * (target_y_ - current_y_);

        current_z_ +=
            gain * (target_z_ - current_z_);

        geometry_msgs::msg::PoseStamped pose;

        pose.header.stamp = now();
        pose.header.frame_id = "tool0";

        pose.pose.position.x =
            current_x_ + disturbance_x_;

        pose.pose.position.y =
            current_y_ + disturbance_y_;

        pose.pose.position.z =
            current_z_ + disturbance_z_;

        pose.pose.orientation.w = 1.0;

        current_pub_->publish(pose);

        /* RCLCPP_INFO(
            get_logger(),
            "Servo current x=%.3f y=%.3f z=%.3f",
            current_x_,
            current_y_,
            current_z_); */
    }

private:
    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr disturbance_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr current_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    double target_x_ = 0.0;
    double target_y_ = 0.0;
    double target_z_ = 0.0;

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_z_ = 0.0;

    double disturbance_x_ = 0.0;
    double disturbance_y_ = 0.0;
    double disturbance_z_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<ToolServoSimulator>());

    rclcpp::shutdown();

    return 0;
}

// end of file: ros2_ws/src/wbp_tool_control/src/tool_servo_simulator.cpp