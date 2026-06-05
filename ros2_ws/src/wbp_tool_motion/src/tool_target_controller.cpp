// start of file: ros2_ws/src/wbp_tool_motion/src/tool_target_controller.cpp
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

class ToolTargetController : public rclcpp::Node
{
public:
    ToolTargetController()
        : Node("tool_target_controller")
    {
        target_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/world_tool_target",
                10,
                std::bind(
                    &ToolTargetController::target_callback,
                    this,
                    std::placeholders::_1));

        tool_cmd_pub_ =
            create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/sim_tool_motion_command",
                10);

        RCLCPP_INFO(
            get_logger(),
            "ToolTargetController started");
    }

private:
    void target_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped cmd;

        cmd.header.stamp = now();

        cmd.header.frame_id = "tool0";

        cmd.pose = msg->pose;

        tool_cmd_pub_->publish(cmd);

        RCLCPP_INFO(
            get_logger(),
            "Published tool motion cmd x=%.3f y=%.3f z=%.3f",
            cmd.pose.position.x,
            cmd.pose.position.y,
            cmd.pose.position.z);
    }

private:
    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr tool_cmd_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<ToolTargetController>());

    rclcpp::shutdown();

    return 0;
}

// end of file: ros2_ws/src/wbp_tool_motion/src/tool_target_controller.cpp