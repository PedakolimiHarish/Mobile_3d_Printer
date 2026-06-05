// start of file: src/wbp_mobile_fabrication/src/world_motion_controller.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <queue>
#include "wbp_sim_interfaces/msg/motion_target.hpp"
#include "wbp_interfaces/msg/base_motion_command.hpp"
#include <nav_msgs/msg/odometry.hpp>
class WorldMotionController : public rclcpp::Node
{
public:
    WorldMotionController()
        : Node("world_motion_controller")
    {
        target_pub_ =
            create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/world_tool_target",
                10);

        completion_pub_ =
            create_publisher<
                std_msgs::msg::UInt32>(
                "/firmware/world_motion_complete",
                10);

        base_command_pub_ =
            create_publisher<
                wbp_interfaces::msg::BaseMotionCommand>(
                "/base/motion_command",
                10);

        tool_pose_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/tool_world_pose",
                10,
                std::bind(
                    &WorldMotionController::toolPoseCallback,
                    this,
                    std::placeholders::_1));

        odom_sub_ =
            create_subscription<
                nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(
                    &WorldMotionController::odomCallback,
                    this,
                    std::placeholders::_1));

        motion_target_sub_ =
            create_subscription<
                wbp_sim_interfaces::msg::MotionTarget>(
                "/sim/motion/target",
                10,
                std::bind(
                    &WorldMotionController::motionTargetCallback,
                    this,
                    std::placeholders::_1));

        error_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/world_tool_error",
                10,
                std::bind(
                    &WorldMotionController::errorCallback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "WorldMotionController started");
    }

private:
    void toolPoseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_tool_pose_ = *msg;
        tool_pose_received_ = true;
    }

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        base_x_ =
            msg->pose.pose.position.x;

        base_y_ =
            msg->pose.pose.position.y;
    }

    void errorCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        latest_error_ = *msg;

        if (!target_active_)
            return;

        double ex = msg->pose.position.x;
        double ey = msg->pose.position.y;
        (void)msg->pose.position.z;

        double norm =
            std::sqrt(ex * ex +
                      ey * ey);

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "WORLD ERROR seq=%u ex=%.3f ey=%.3f norm=%.3f",
            active_seq_,
            ex,
            ey,
            norm);

        constexpr double COMPLETE_THRESHOLD = 0.10;
        constexpr double COMPLETE_TIME_SEC = 0.5;

        if (norm < COMPLETE_THRESHOLD)
        {

            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                200,
                "INSIDE THRESHOLD norm=%.3f",
                norm);

            if (!within_threshold_)
            {
                within_threshold_ = true;
                within_threshold_start_ = now();
            }

            double dwell_time =
                (now() - within_threshold_start_).seconds();

            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                200,
                "DWELL %.3f",
                dwell_time);

            if (dwell_time >= COMPLETE_TIME_SEC)
            {
                std_msgs::msg::UInt32 done;

                done.data = active_seq_;

                RCLCPP_WARN(
                    get_logger(),
                    "COMPLETE TOOL=(%.3f %.3f) TARGET=(%.3f %.3f) ERROR=(%.3f %.3f)",
                    current_tool_pose_.pose.position.x,
                    current_tool_pose_.pose.position.y,
                    active_target_x_,
                    active_target_y_,
                    ex,
                    ey);

                completion_pub_->publish(done);

                RCLCPP_WARN(
                    get_logger(),
                    "COMPLETE seq=%u norm=%.4f dwell=%.3f",
                    active_seq_,
                    norm,
                    dwell_time);

                if (!pending_targets_.empty())
                {
                    pending_targets_.pop();
                }

                if (!pending_targets_.empty())
                {
                    active_seq_ =
                        pending_targets_.front().seq;

                    target_active_ = true;
                }
                else
                {
                    target_active_ = false;
                }

                within_threshold_ = false;
            }
        }
        else
        {
            if (within_threshold_)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "THRESHOLD LOST");
            }

            within_threshold_ = false;
        }
    }

    void motionTargetCallback(const wbp_sim_interfaces::msg::MotionTarget::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped target;

        target.header.stamp = now();
        target.header.frame_id = "odom";

        target.pose.position.x = msg->x;
        target.pose.position.y = msg->y;
        target.pose.position.z = msg->z;

        active_target_x_ = msg->x;
        active_target_y_ = msg->y;

        if (tool_pose_received_)
        {
            double dx =
                msg->x -
                base_x_;

            double dy =
                msg->y -
                base_y_;

            double distance =
                std::sqrt(dx * dx + dy * dy);

            constexpr double WORK_RADIUS = 0.75;

            if (distance > WORK_RADIUS)
            {
                wbp_interfaces::msg::BaseMotionCommand cmd;

                cmd.sequence_id =
                    msg->sequence_id;

                cmd.target_x =
                    msg->x -
                    WORK_RADIUS * dx / distance;

                cmd.target_y =
                    msg->y -
                    WORK_RADIUS * dy / distance;

                cmd.target_theta = 0.0;

                cmd.linear_velocity = 0.15;
                cmd.angular_velocity = 0.15;

                RCLCPP_WARN(
                    get_logger(),
                    "BASE CMD seq=%u target=(%.3f %.3f) world=(%.3f %.3f) dist=%.3f",
                    cmd.sequence_id,
                    cmd.target_x,
                    cmd.target_y,
                    msg->x,
                    msg->y,
                    distance);

                base_command_pub_->publish(cmd);
            }
        }

        target.pose.orientation.x = 0.0;
        target.pose.orientation.y = 0.0;
        target.pose.orientation.z = 0.0;
        target.pose.orientation.w = 1.0;

        PendingTarget target_entry;

        target_entry.seq =
            msg->sequence_id;

        pending_targets_.push(target_entry);

        RCLCPP_WARN(
            get_logger(),
            "PUSH seq=%u queue=%zu",
            msg->sequence_id,
            pending_targets_.size());

        if (!target_active_)
        {
            active_seq_ =
                pending_targets_.front().seq;

            target_active_ = true;
        }

        target_pub_->publish(target);
    }

private:
    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;

    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr completion_pub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr error_sub_;

    rclcpp::Subscription<
        wbp_sim_interfaces::msg::MotionTarget>::SharedPtr motion_target_sub_;

    rclcpp::Publisher<
        wbp_interfaces::msg::BaseMotionCommand>::SharedPtr
        base_command_pub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr
        tool_pose_sub_;

    rclcpp::Subscription<
        nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;

    geometry_msgs::msg::PoseStamped current_tool_pose_;

    double base_x_ = 0.0;
    double base_y_ = 0.0;

    double active_target_x_ = 0.0;
    double active_target_y_ = 0.0;

    bool tool_pose_received_ = false;
    struct PendingTarget
    {
        uint32_t seq;
    };

    rclcpp::Time within_threshold_start_;
    bool within_threshold_ = false;

    std::queue<PendingTarget> pending_targets_;

    geometry_msgs::msg::PoseStamped latest_error_;

    uint32_t active_seq_ = 0;

    bool target_active_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WorldMotionController>());
    rclcpp::shutdown();
    return 0;
}

// end of file : src / wbp_mobile_fabrication / src / world_motion_controller.cpp