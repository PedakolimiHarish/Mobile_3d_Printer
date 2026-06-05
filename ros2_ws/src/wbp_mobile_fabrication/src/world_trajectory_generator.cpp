// start of file: ros2_ws/src/wbp_mobile_fabrication/src/world_trajectory_generator.cpp
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

class WorldTrajectoryGenerator : public rclcpp::Node
{
public:
    WorldTrajectoryGenerator()
        : Node("world_trajectory_generator")
    {
        target_pub_ =
            create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/world_tool_target",
                10);

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &WorldTrajectoryGenerator::update,
                    this));

        RCLCPP_INFO(
            get_logger(),
            "WorldTrajectoryGenerator started");
    }

private:
    void update()
    {
        geometry_msgs::msg::PoseStamped target;

        target.header.stamp = now();

        target.header.frame_id = "map";

        // =====================================
        // FIXED HOLD POSITION TARGET
        // =====================================

        target.pose.position.x = 0.6;

        target.pose.position.y = 0.0;

        target.pose.position.z = 0.265;

        target.pose.orientation.w = 1.0;

        target_pub_->publish(target);
    }

private:
    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    double trajectory_x_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<
            WorldTrajectoryGenerator>());

    rclcpp::shutdown();

    return 0;
}
// end of file: ros2_ws/src/wbp_mobile_fabrication/src/world_trajectory_generator.cpp