// start of file ros2_ws/src/wbp_localization/src/anchor_localizer.cpp

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "wbp_interfaces/msg/anchor_correction.hpp"

class AnchorLocalizer : public rclcpp::Node
{
public:
    AnchorLocalizer()
        : Node("anchor_localizer")
    {

        tf_broadcaster_ =
            std::make_shared<
                tf2_ros::TransformBroadcaster>(this);

        correction_sub_ =
            create_subscription<
                wbp_interfaces::msg::AnchorCorrection>(
                "/anchor_correction",
                10,
                std::bind(
                    &AnchorLocalizer::correction_callback,
                    this,
                    std::placeholders::_1));

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(
                    &AnchorLocalizer::publish_transform,
                    this));

        RCLCPP_INFO(
            get_logger(),
            "AnchorLocalizer started");
    }

private:
    void publish_transform()
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now();
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "odom";

        // =====================================
        // INITIAL ZERO CORRECTION
        // =====================================

        tf_msg.transform.translation.x = 0.0;
        tf_msg.transform.translation.y = 0.0;
        tf_msg.transform.translation.z = 0.0;

        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;

        tf_msg.transform.rotation.z = 0.0;
        tf_msg.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    void correction_callback(
        const wbp_interfaces::msg::AnchorCorrection::SharedPtr msg)
    {
        (void)msg;
    }

private:
    std::shared_ptr<
        tf2_ros::TransformBroadcaster>
        tf_broadcaster_;

    rclcpp::Subscription<
        wbp_interfaces::msg::AnchorCorrection>::SharedPtr correction_sub_;

    double correction_x_ = 0.0;
    double correction_y_ = 0.0;
    double correction_theta_ = 0.0;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<AnchorLocalizer>());

    rclcpp::shutdown();

    return 0;
}

// end of file ros2_ws/src/wbp_localization/src/anchor_localizer.cpp