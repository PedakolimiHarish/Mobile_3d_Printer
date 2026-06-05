// start of file: ros2_ws/src/wbp_tool_control/src/tool_tracking_monitor.cpp
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

class ToolTrackingMonitor : public rclcpp::Node
{
public:
    ToolTrackingMonitor()
        : Node("tool_tracking_monitor")
    {
        target_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/tool_target_pose",
                10,
                std::bind(
                    &ToolTrackingMonitor::target_callback,
                    this,
                    std::placeholders::_1));

        current_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/tool_world_pose",
                10,
                std::bind(
                    &ToolTrackingMonitor::current_callback,
                    this,
                    std::placeholders::_1));

        error_pub_ =
            create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/tool_tracking_error",
                10);

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &ToolTrackingMonitor::publish_error,
                    this));

        RCLCPP_INFO(
            get_logger(),
            "ToolTrackingMonitor started");
    }

private:
    void target_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_ = *msg;
    }

    void current_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_ = *msg;
    }

    void publish_error()
    {
        geometry_msgs::msg::PoseStamped error;

        error.header.stamp = now();
        error.header.frame_id = "map";

        error.pose.position.x =
            target_.pose.position.x -
            current_.pose.position.x;

        error.pose.position.y =
            target_.pose.position.y -
            current_.pose.position.y;

        error.pose.position.z =
            target_.pose.position.z -
            current_.pose.position.z;

        error.pose.orientation.w = 1.0;

        error_pub_->publish(error);
    }

private:
    geometry_msgs::msg::PoseStamped target_;
    geometry_msgs::msg::PoseStamped current_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr current_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr error_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<ToolTrackingMonitor>());

    rclcpp::shutdown();

    return 0;
}

// end of file: ros2_ws/src/wbp_tool_control/src/tool_tracking_monitor.cpp