// start of file: ros2_ws/src/wbp_sim_motion/src/execution_tf_publisher.cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "wbp_interfaces/msg/machine_state.hpp"

class ExecutionTFPublisher : public rclcpp::Node
{
public:
    ExecutionTFPublisher()
        : Node("execution_tf_publisher")
    {
        // Parameters (prototype mapping)
        segment_dx_ = this->declare_parameter<double>("segment_dx", 0.10);
        layer_height_ = this->declare_parameter<double>("layer_height", 0.02);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        state_sub_ = this->create_subscription<wbp_interfaces::msg::MachineState>(
            "/machine_state",
            10,
            std::bind(&ExecutionTFPublisher::on_state, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Execution TF Publisher started (segment_dx=%.3f, layer_height=%.3f)",
            segment_dx_,
            layer_height_);
    }

private:
    void on_state(const wbp_interfaces::msg::MachineState::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped tf;

        tf.header.stamp = this->get_clock()->now();
        tf.header.frame_id = "base_link";
        tf.child_frame_id = "toolhead_link";

        tf.transform.translation.x = msg->segment * segment_dx_;
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = msg->layer * layer_height_;

        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;
        tf.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(tf);
    }

    // ROS
    rclcpp::Subscription<wbp_interfaces::msg::MachineState>::SharedPtr state_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Parameters
    double segment_dx_;
    double layer_height_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExecutionTFPublisher>());
    rclcpp::shutdown();
    return 0;
}

// end of file
