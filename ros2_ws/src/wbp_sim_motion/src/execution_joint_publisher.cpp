// start of file ros2_ws/src/wbp_sim_motion/src/execution_joint_publisher.cpp
#include <rclcpp/rclcpp.hpp>
#include "wbp_interfaces/msg/machine_state.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class ExecutionJointPublisher : public rclcpp::Node
{
public:
    ExecutionJointPublisher()
        : Node("execution_joint_publisher")
    {
        layer_height_ = this->declare_parameter<double>("layer_height", 0.025);
        segment_dx_ = this->declare_parameter<double>("segment_dx", 0.10);

        trajectory_pub_ =
            this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/robot_controller/joint_trajectory", 10);

        state_sub_ =
            this->create_subscription<wbp_interfaces::msg::MachineState>(
                "/machine_state",
                10,
                std::bind(&ExecutionJointPublisher::on_state, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "Execution Joint Publisher started (layer_height=%.3f, segment_dx=%.3f)",
                    layer_height_, segment_dx_);
    }

private:
    void on_state(const wbp_interfaces::msg::MachineState::SharedPtr msg)
    {
        trajectory_msgs::msg::JointTrajectory traj;

        traj.joint_names = {
            "mast_to_carriage_joint",
            "carriage_to_tool_joint",
            "tool_mount_joint"};

        trajectory_msgs::msg::JointTrajectoryPoint point;

        point.positions = {
            msg->layer * layer_height_,
            msg->segment * segment_dx_,
            0.0};

        point.time_from_start = rclcpp::Duration::from_seconds(0.1);

        traj.points.push_back(point);
        traj.header.stamp = now();

        trajectory_pub_->publish(traj);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Subscription<wbp_interfaces::msg::MachineState>::SharedPtr state_sub_;

    double layer_height_;
    double segment_dx_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExecutionJointPublisher>());
    rclcpp::shutdown();
    return 0;
}

// end of file ros2_ws/src/wbp_sim_motion/src/execution_joint_publisher.cpp