// start of file: ros2_ws/src/wbp_base_motion/src/base_motion_controller.cpp
#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include "wbp_interfaces/msg/base_motion_command.hpp"
#include "wbp_interfaces/msg/base_motion_result.hpp"
#include "wbp_interfaces/msg/base_state.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

struct ActiveCommand
{
    uint32_t seq;

    double tx;
    double ty;
    double ttheta;

    double linear_vel;
    double angular_vel;
};

class BaseMotionController : public rclcpp::Node
{
public:
    BaseMotionController()
        : Node("base_motion_controller")
    {
        command_sub_ =
            create_subscription<wbp_interfaces::msg::BaseMotionCommand>(
                "/base/motion_command",
                100,
                std::bind(
                    &BaseMotionController::command_callback,
                    this,
                    std::placeholders::_1));

        result_pub_ =
            create_publisher<wbp_interfaces::msg::BaseMotionResult>(
                "/base/motion_result",
                100);

        state_pub_ =
            create_publisher<wbp_interfaces::msg::BaseState>(
                "/base_state",
                100);

        cmd_vel_pub_ =
            create_publisher<
                geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10);

        odom_sub_ =
            create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                50,
                std::bind(
                    &BaseMotionController::odom_callback,
                    this,
                    std::placeholders::_1));

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ =
            create_wall_timer(
                20ms,
                std::bind(
                    &BaseMotionController::update,
                    this));

        RCLCPP_INFO(get_logger(), "BaseMotionController started");
    }

private:
    void command_callback(
        const wbp_interfaces::msg::BaseMotionCommand::SharedPtr msg)
    {
        ActiveCommand cmd;

        cmd.seq = msg->sequence_id;

        cmd.tx = msg->target_x;
        cmd.ty = msg->target_y;
        cmd.ttheta = msg->target_theta;

        cmd.linear_vel = msg->linear_velocity;
        cmd.angular_vel = msg->angular_velocity;

        queue_.push(cmd);

        RCLCPP_INFO(
            get_logger(),
            "QUEUE SIZE=%zu",
            queue_.size());

        publish_result(
            cmd.seq,
            wbp_interfaces::msg::BaseMotionResult::ACCEPTED);

        RCLCPP_INFO(
            get_logger(),
            "Queued base command seq=%u",
            cmd.seq);
    }

    void publish_odom_tf()
    {
        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.stamp =
            odom_stamp_;

        tf_msg.header.frame_id =
            "odom";

        tf_msg.child_frame_id =
            "base_footprint";

        tf_msg.transform.translation.x =
            current_x_;

        tf_msg.transform.translation.y =
            current_y_;

        tf_msg.transform.translation.z =
            0.0;

        double half_yaw =
            current_theta_ * 0.5;

        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = std::sin(half_yaw);
        tf_msg.transform.rotation.w = std::cos(half_yaw);

        tf_broadcaster_->sendTransform(tf_msg);
    }

    void update()
    {

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "UPDATE active=%d queue=%zu",
            active_,
            queue_.size());

        constexpr double dt = 0.02;

        if (!active_ && !queue_.empty())
        {
            RCLCPP_INFO(
                get_logger(),
                "STARTING COMMAND");

            current_ = queue_.front();
            queue_.pop();

            active_ = true;
            elapsed_time_ = 0.0;

            stopping_ = false;

            target_vx_ =
                current_.linear_vel;

            target_wz_ =
                current_.angular_vel;

            publish_result(
                current_.seq,
                wbp_interfaces::msg::BaseMotionResult::IN_PROGRESS);
        }

        if (active_)
        {
            execute_active(dt);
        }

        publish_state();
    }

    void execute_active(double dt)
    {
        elapsed_time_ += dt;

        if (stopping_)
        {
            target_vx_ = 0.0;
            target_wz_ = 0.0;
        }

        /* =========================
           Compute pose errors FIRST
           ========================= */

        double dx =
            current_.tx - current_x_;

        double dy =
            current_.ty - current_y_;

        double dist_error =
            std::sqrt(dx * dx + dy * dy);

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "BASE TRACK seq=%u target=(%.3f %.3f) current=(%.3f %.3f) dx=%.3f dy=%.3f dist=%.3f theta=%.3f",
            current_.seq,
            current_.tx,
            current_.ty,
            current_x_,
            current_y_,
            dx,
            dy,
            dist_error,
            current_theta_);
        /* =========================
           Signed forward error
           ========================= */

        double forward_error =
            dx * std::cos(current_theta_) +
            dy * std::sin(current_theta_);

        double desired_heading =
            std::atan2(dy, dx);

        double yaw_error =
            desired_heading - current_theta_;

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "TX=%.3f CUR=%.3f DX=%.3f FORWARD=%.3f DIST=%.3f",
            current_.tx,
            current_x_,
            dx,
            forward_error,
            dist_error);

        /* Normalize yaw error */

        while (yaw_error > M_PI)
        {
            yaw_error -= 2.0 * M_PI;
        }

        while (yaw_error < -M_PI)
        {
            yaw_error += 2.0 * M_PI;
        }

        /* =========================
           Generate target velocities
           ========================= */

        if (dist_error > 0.05)
        {
            target_wz_ =
                std::clamp(
                    yaw_error * 2.0,
                    -0.5,
                    0.5);

            if (std::abs(yaw_error) < 0.5)
            {
                target_vx_ =
                    std::clamp(
                        forward_error * 0.8,
                        -0.4,
                        0.4);
            }
            else
            {
                target_vx_ = 0.0;
            }
        }
        else
        {
            target_vx_ = 0.0;
            target_wz_ = 0.0;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "TX=%.3f CUR=%.3f DX=%.3f DIST=%.3f TVX=%.3f CVX=%.3f",
            current_.tx,
            current_x_,
            dx,
            dist_error,
            target_vx_,
            current_vx_);

        /* =========================
           Smooth velocities
           ========================= */

        constexpr double accel_rate = 0.03;

        current_vx_ +=
            (target_vx_ - current_vx_) *
            accel_rate;

        current_wz_ +=
            (target_wz_ - current_wz_) *
            accel_rate;

        /* =========================
           Publish cmd_vel
           ========================= */

        geometry_msgs::msg::Twist cmd;

        cmd.linear.x =
            current_vx_;

        cmd.angular.z =
            current_wz_;

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "TARGET_VX=%.3f CURRENT_VX=%.3f",
            target_vx_,
            current_vx_);

        cmd_vel_pub_->publish(cmd);

        if (dist_error < 0.05)
        {
            stopping_ = true;
        }

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "STOPPING dist=%.3f yaw=%.3f vx=%.3f wz=%.3f",
            dist_error,
            yaw_error,
            current_vx_,
            current_wz_);

        if (stopping_ &&
            std::abs(current_vx_) < 0.05 &&
            std::abs(current_wz_) < 0.05)
        {
            active_ = false;
            stopping_ = false;

            geometry_msgs::msg::Twist stop_cmd;

            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;

            cmd_vel_pub_->publish(stop_cmd);

            RCLCPP_WARN(
                get_logger(),
                "STEP A");

            publish_result(
                current_.seq,
                wbp_interfaces::msg::
                    BaseMotionResult::COMPLETED);

            RCLCPP_WARN(
                get_logger(),
                "STEP B");

            RCLCPP_INFO(
                get_logger(),
                "Completed base seq=%u",
                current_.seq);

            RCLCPP_WARN(
                get_logger(),
                "STEP C");

            return;
        }
    }

    void publish_result(uint32_t seq, uint8_t status)
    {
        wbp_interfaces::msg::BaseMotionResult msg;

        msg.sequence_id = seq;
        msg.status = status;

        result_pub_->publish(msg);
    }

    void publish_state()
    {
        wbp_interfaces::msg::BaseState msg;

        msg.x = current_x_;
        msg.y = current_y_;
        msg.theta = current_theta_;

        msg.vx = current_vx_;
        msg.vy = 0.0;
        msg.omega = current_wz_;

        msg.localized = true;

        state_pub_->publish(msg);
    }
    void odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ =
            msg->pose.pose.position.x;

        current_y_ =
            msg->pose.pose.position.y;

        double qw =
            msg->pose.pose.orientation.w;

        double qz =
            msg->pose.pose.orientation.z;

        double qx =
            msg->pose.pose.orientation.x;

        double qy =
            msg->pose.pose.orientation.y;

        /* Proper yaw extraction */

        double siny_cosp =
            2.0 * (qw * qz + qx * qy);

        double cosy_cosp =
            1.0 - 2.0 * (qy * qy + qz * qz);

        current_theta_ =
            std::atan2(siny_cosp, cosy_cosp);

        odom_stamp_ = msg->header.stamp;

        publish_odom_tf();
    }

private:
    rclcpp::Subscription<wbp_interfaces::msg::BaseMotionCommand>::SharedPtr command_sub_;
    rclcpp::Publisher<wbp_interfaces::msg::BaseMotionResult>::SharedPtr result_pub_;
    rclcpp::Publisher<wbp_interfaces::msg::BaseState>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Time odom_stamp_;

private:
    std::queue<ActiveCommand> queue_;

    ActiveCommand current_;

    bool active_ = false;
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;

    double elapsed_time_ = 0.0;

    double current_vx_ = 0.0;
    double current_wz_ = 0.0;

    double target_vx_ = 0.0;
    double target_wz_ = 0.0;

    bool stopping_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<BaseMotionController>());

    rclcpp::shutdown();

    return 0;
}

// end of file: ros2_ws/src/wbp_base_motion/src/base_motion_controller.cpp