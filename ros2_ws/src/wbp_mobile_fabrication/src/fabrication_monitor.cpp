// start of file: ros2_ws/src/wbp_mobile_fabrication/src/fabrication_monitor.cpp
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "wbp_interfaces/msg/anchor_correction.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "wbp_interfaces/msg/anchor_correction.hpp"
#include "wbp_interfaces/msg/tool_compensation.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <wbp_sim_interfaces/msg/motion_target.hpp>
#include <cmath>

class FabricationMonitor : public rclcpp::Node
{
public:
    FabricationMonitor()
        : Node("fabrication_monitor")
    {

        target_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/world_tool_target",
                10,
                std::bind(
                    &FabricationMonitor::target_callback,
                    this,
                    std::placeholders::_1));

        error_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/world_tool_error",
                10,
                std::bind(
                    &FabricationMonitor::error_callback,
                    this,
                    std::placeholders::_1));

        world_pose_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/tool_world_pose",
                10,
                std::bind(
                    &FabricationMonitor::world_pose_callback,
                    this,
                    std::placeholders::_1));

        cmd_vel_sub_ =
            create_subscription<
                geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10,
                std::bind(
                    &FabricationMonitor::cmdVelCallback,
                    this,
                    std::placeholders::_1));

        compensation_sub_ =
            create_subscription<
                wbp_interfaces::msg::ToolCompensation>(
                "/tool_compensation_offset",
                10,
                std::bind(
                    &FabricationMonitor::compensation_callback,
                    this,
                    std::placeholders::_1));

        anchor_sub_ =
            create_subscription<
                wbp_interfaces::msg::AnchorCorrection>(
                "/anchor_correction",
                10,
                std::bind(
                    &FabricationMonitor::anchor_callback,
                    this,
                    std::placeholders::_1));

        odom_sub_ =
            create_subscription<
                nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(
                    &FabricationMonitor::odom_callback,
                    this,
                    std::placeholders::_1));

        joint_state_sub_ =
            create_subscription<
                sensor_msgs::msg::JointState>(
                "/joint_states",
                10,
                std::bind(
                    &FabricationMonitor::joint_state_callback,
                    this,
                    std::placeholders::_1));

        tool_cmd_sub_ =
            this->create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/sim_tool_motion_command",
                10,
                std::bind(
                    &FabricationMonitor::toolCmdCallback,
                    this,
                    std::placeholders::_1));

        motion_target_sub_ =
            create_subscription<
                wbp_sim_interfaces::msg::MotionTarget>(
                "/sim/motion/target",
                100,
                std::bind(
                    &FabricationMonitor::motion_target_callback,
                    this,
                    std::placeholders::_1));

        world_complete_sub_ =
            create_subscription<
                std_msgs::msg::UInt32>(
                "/firmware/world_motion_complete",
                100,
                std::bind(
                    &FabricationMonitor::world_complete_callback,
                    this,
                    std::placeholders::_1));

        arm_cmd_sub_ =
            create_subscription<
                std_msgs::msg::Float64MultiArray>(
                "/arm_controller/commands",
                100,
                std::bind(
                    &FabricationMonitor::arm_cmd_callback,
                    this,
                    std::placeholders::_1));

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(200),
                std::bind(
                    &FabricationMonitor::print_status,
                    this));

        RCLCPP_INFO(
            get_logger(),
            "FabricationMonitor started");
    }

private:
    void target_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_pose_ = *msg;
    }

    void motion_target_callback(
        const wbp_sim_interfaces::msg::MotionTarget::SharedPtr msg)
    {
        active_seq_ =
            msg->sequence_id;
    }

    void world_complete_callback(
        const std_msgs::msg::UInt32::SharedPtr msg)
    {
        last_completed_seq_ =
            msg->data;
    }

    void arm_cmd_callback(
        const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        last_arm_cmd_ = *msg;
    }

    void cmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_vel_ = *msg;
    }

    void error_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        error_pose_ = *msg;
    }

    void anchor_callback(
        const wbp_interfaces::msg::AnchorCorrection::SharedPtr msg)
    {
        anchor_correction_ = *msg;
    }

    void world_pose_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        world_pose_ = *msg;
    }

    void compensation_callback(
        const wbp_interfaces::msg::ToolCompensation::SharedPtr msg)
    {
        compensation_ = *msg;
    }

    void joint_state_callback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint_states_ = *msg;
    }

    void toolCmdCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_tool_cmd_ = *msg;
    }

    void odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = *msg;
    }

    void print_status()
    {
        std::stringstream ss;

        ss << std::fixed
           << std::setprecision(3);

        double error_norm =
            std::sqrt(
                error_pose_.pose.position.x *
                    error_pose_.pose.position.x +

                error_pose_.pose.position.y *
                    error_pose_.pose.position.y +

                error_pose_.pose.position.z *
                    error_pose_.pose.position.z);

        ss << "FAB | ";

        ss << "SEQ["
           << active_seq_
           << "] | ";

        ss << "DONE["
           << last_completed_seq_
           << "] | ";

        ss << "NORM["
           << error_norm
           << "] | ";

        ss << "TARGET["
           << target_pose_.pose.position.x << " "
           << target_pose_.pose.position.y << " "
           << target_pose_.pose.position.z << "] | ";

        ss << "WORLD["
           << world_pose_.pose.position.x << " "
           << world_pose_.pose.position.y << " "
           << world_pose_.pose.position.z << "] | ";

        ss << "ERROR["
           << error_pose_.pose.position.x << " "
           << error_pose_.pose.position.y << " "
           << error_pose_.pose.position.z << "] | ";

        ss << "COMP["
           << compensation_.dx << " "
           << compensation_.dy << " "
           << compensation_.dz << "] | ";

        ss << "ARM_CMD[";

        for (size_t i = 0;
             i < last_arm_cmd_.data.size();
             ++i)
        {
            ss << last_arm_cmd_.data[i];

            if (i + 1 < last_arm_cmd_.data.size())
            {
                ss << " ";
            }
        }

        ss << "] | ";

        ss << "ANCHOR["
           << anchor_correction_.x << " "
           << anchor_correction_.y << " "
           << anchor_correction_.theta << "] | ";

        ss << "CMD["
           << last_tool_cmd_.pose.position.x << " "
           << last_tool_cmd_.pose.position.y << " "
           << last_tool_cmd_.pose.position.z << "] | ";

        ss << "CMD_VEL["
           << last_cmd_vel_.linear.x << " "
           << last_cmd_vel_.linear.y << " "
           << last_cmd_vel_.angular.z << "] | ";

        ss << "JOINTS[";
        for (size_t i = 0;
             i < joint_states_.name.size();
             ++i)
        {
            ss << joint_states_.name[i]
               << "=";

            if (i < joint_states_.position.size())
            {
                ss << joint_states_.position[i];
            }
            else
            {
                ss << "NA";
            }

            if (i + 1 < joint_states_.name.size())
            {
                ss << " ";
            }
        }
        ss << "] | ";

        double qw =
            odom_.pose.pose.orientation.w;

        double qx =
            odom_.pose.pose.orientation.x;

        double qy =
            odom_.pose.pose.orientation.y;

        double qz =
            odom_.pose.pose.orientation.z;

        double siny_cosp =
            2.0 * (qw * qz + qx * qy);

        double cosy_cosp =
            1.0 - 2.0 * (qy * qy + qz * qz);

        double yaw =
            2.0 * std::atan2(qz, qw);

        ss << "BASE["
           << odom_.pose.pose.position.x << " "
           << odom_.pose.pose.position.y << " "
           << yaw
           << "]";

        RCLCPP_INFO(
            get_logger(),
            "%s",
            ss.str().c_str());
    }

private:
    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr error_sub_;

    rclcpp::Subscription<
        wbp_interfaces::msg::ToolCompensation>::SharedPtr compensation_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr tool_cmd_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr world_pose_sub_;

    rclcpp::Subscription<
        wbp_interfaces::msg::AnchorCorrection>::SharedPtr anchor_sub_;

    rclcpp::Subscription<
        sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    rclcpp::Subscription<
        nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Subscription<
        wbp_sim_interfaces::msg::MotionTarget>::SharedPtr
        motion_target_sub_;

    rclcpp::Subscription<
        std_msgs::msg::UInt32>::SharedPtr
        world_complete_sub_;

    rclcpp::Subscription<
        std_msgs::msg::Float64MultiArray>::SharedPtr
        arm_cmd_sub_;

    std_msgs::msg::Float64MultiArray
        last_arm_cmd_;

    uint32_t active_seq_ = 0;

    uint32_t last_completed_seq_ = 0;

    geometry_msgs::msg::Twist last_cmd_vel_;

    wbp_interfaces::msg::AnchorCorrection anchor_correction_;

    geometry_msgs::msg::PoseStamped target_pose_;

    geometry_msgs::msg::PoseStamped error_pose_;

    geometry_msgs::msg::PoseStamped last_tool_cmd_;

    nav_msgs::msg::Odometry odom_;

    geometry_msgs::msg::PoseStamped world_pose_;

    wbp_interfaces::msg::ToolCompensation compensation_;

    sensor_msgs::msg::JointState joint_states_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<
            FabricationMonitor>());

    rclcpp::shutdown();

    return 0;
}

// end of file: ros2_ws/src/wbp_mobile_fabrication/src/fabrication_monitor.cpp