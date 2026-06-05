// start of file: ros2_ws/src/wbp_compensation/src/world_compensation_controller.cpp

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "wbp_interfaces/msg/tool_compensation.hpp"

class WorldCompensationController : public rclcpp::Node
{
public:
    WorldCompensationController()
        : Node("world_compensation_controller")
    {
        error_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/world_tool_error",
                10,
                std::bind(
                    &WorldCompensationController::error_callback,
                    this,
                    std::placeholders::_1));

        compensation_pub_ =
            create_publisher<
                wbp_interfaces::msg::ToolCompensation>(
                "/tool_compensation_offset",
                10);

        RCLCPP_INFO(
            get_logger(),
            "WorldCompensationController started");
    }

private:
    void error_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        constexpr double kp = 0.2;
        constexpr double kd = 0.02;

        double error_x =
            msg->pose.position.x;

        double error_y =
            msg->pose.position.y;

        double error_z =
            msg->pose.position.z;

        double derivative_x =
            error_x - prev_error_x_;

        double derivative_y =
            error_y - prev_error_y_;

        double derivative_z =
            error_z - prev_error_z_;

        prev_error_x_ = error_x;
        prev_error_y_ = error_y;
        prev_error_z_ = error_z;

        wbp_interfaces::msg::ToolCompensation correction;

        correction.dx =
            kp * error_x +
            kd * derivative_x;

        correction.dy =
            kp * error_y +
            kd * derivative_y;

        correction.dz =
            kp * error_z +
            kd * derivative_z;

        correction.valid = true;

        compensation_pub_->publish(correction);
    }

private:
    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr error_sub_;

    rclcpp::Publisher<
        wbp_interfaces::msg::ToolCompensation>::SharedPtr compensation_pub_;

    double prev_error_x_ = 0.0;
    double prev_error_y_ = 0.0;
    double prev_error_z_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<WorldCompensationController>());

    rclcpp::shutdown();

    return 0;
}

// end of file ros2_ws/src/wbp_compensation/src/world_compensation_controller.cpp