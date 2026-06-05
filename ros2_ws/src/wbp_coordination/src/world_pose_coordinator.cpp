// start of file: ros2_ws/src/wbp_coordination/src/world_pose_coordinator.cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class WorldPoseCoordinator : public rclcpp::Node
{
public:
    WorldPoseCoordinator()
        : Node("world_pose_coordinator"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {

        tool_pose_pub_ =
            create_publisher<geometry_msgs::msg::PoseStamped>(
                "/tool_world_pose",
                100);

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(
                    &WorldPoseCoordinator::update,
                    this));

        RCLCPP_INFO(
            get_logger(),
            "WorldPoseCoordinator started");
    }

private:
    void update()
    {
        geometry_msgs::msg::TransformStamped tf_msg;

        try
        {
            tf_msg =
                tf_buffer_.lookupTransform(
                    "odom",
                    "tool_link",
                    tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "TF unavailable: %s",
                ex.what());

            return;
        }

        geometry_msgs::msg::PoseStamped pose;

        pose.header = tf_msg.header;

        pose.pose.position.x =
            tf_msg.transform.translation.x;

        pose.pose.position.y =
            tf_msg.transform.translation.y;

        pose.pose.position.z =
            tf_msg.transform.translation.z;

        pose.pose.orientation =
            tf_msg.transform.rotation;

        tool_pose_pub_->publish(pose);
    }

private:
    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr tool_pose_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

private:
    tf2_ros::Buffer tf_buffer_;

    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<WorldPoseCoordinator>());

    rclcpp::shutdown();

    return 0;
}

// end of file: ros2_ws/src/wbp_coordination/src/world_pose_coordinator.cpp