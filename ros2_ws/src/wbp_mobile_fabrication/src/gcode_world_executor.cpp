// start of file: ros2_ws/src/wbp_mobile_fabrication/src/gcode_world_executor.cpp
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

struct GCodePoint
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

class GcodeWorldExecutor : public rclcpp::Node
{
public:
    GcodeWorldExecutor()
        : Node("gcode_world_executor")
    {
        target_pub_ =
            create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/world_tool_target",
                10);

        world_pose_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/tool_world_pose",
                10,
                std::bind(
                    &GcodeWorldExecutor::worldPoseCallback,
                    this,
                    std::placeholders::_1));

        error_sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/world_tool_error",
                10,
                std::bind(
                    &GcodeWorldExecutor::
                        error_callback,
                    this,
                    std::placeholders::_1));

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(
                    &GcodeWorldExecutor::update,
                    this));

        load_gcode(
            "/home/guru/wbp_controller/test_1.gcode");

        RCLCPP_INFO(
            get_logger(),
            "GcodeWorldExecutor started");
    }

private:
    void load_gcode(const std::string &file)
    {
        std::ifstream in(file);

        if (!in.is_open())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to open G-code file");

            return;
        }

        std::string line;

        double last_x = 0.0;
        double last_y = 0.0;
        double last_z = 0.265;

        while (std::getline(in, line))
        {
            RCLCPP_INFO(
                get_logger(),
                "RAW LINE: [%s]",
                line.c_str());

            if (line.empty())
                continue;

            if (line[0] == ';')
                continue;

            if (line.rfind("G0", 0) != 0 &&
                line.rfind("G1", 0) != 0)
            {
                continue;
            }

            std::stringstream ss(line);

            std::string token;

            GCodePoint p;

            p.x = last_x;
            p.y = last_y;
            p.z = last_z;

            while (ss >> token)
            {
                if (token[0] == 'X')
                {
                    p.x =
                        std::stod(
                            token.substr(1));
                }

                else if (token[0] == 'Y')
                {
                    p.y =
                        std::stod(
                            token.substr(1));
                }

                else if (token[0] == 'Z')
                {
                    p.z =
                        std::stod(
                            token.substr(1));
                }
            }
            RCLCPP_INFO(
                get_logger(),
                "PARSED POINT: x=%.3f y=%.3f z=%.3f",
                p.x,
                p.y,
                p.z);

            points_.push_back(p);

            last_x = p.x;
            last_y = p.y;
            last_z = p.z;
        }

        RCLCPP_INFO(
            get_logger(),
            "Loaded %zu G-code points",
            points_.size());
    }

    void worldPoseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_world_pose_ = *msg;
    }

    void error_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        world_error_ = *msg;
        error_received_ = true;
    }

    void update()
    {

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "EXECUTOR UPDATE");

        if (current_index_ >= points_.size())
        {
            if (!job_complete_)
            {
                RCLCPP_INFO(
                    get_logger(),
                    "WORLD JOB COMPLETE");

                job_complete_ = true;
            }

            return;
        }

        if (!segment_initialized_)
        {
            if (current_index_ + 1 >= points_.size())
            {
                return;
            }

            segment_start_ = points_[current_index_];
            segment_end_ = points_[current_index_ + 1];
            progress_ = 0.0;
            segment_initialized_ = true;
        }

        geometry_msgs::msg::PoseStamped target;

        target.header.stamp = now();

        target.header.frame_id = "odom";

        double interp_x =
            segment_start_.x +
            progress_ *
                (segment_end_.x -
                 segment_start_.x);

        double interp_y =
            segment_start_.y +
            progress_ *
                (segment_end_.y -
                 segment_start_.y);

        double interp_z =
            segment_start_.z +
            progress_ *
                (segment_end_.z -
                 segment_start_.z);

        target.pose.position.x =
            interp_x;

        target.pose.position.y =
            interp_y;

        target.pose.position.z =
            interp_z;

        target.pose.orientation.w = 1.0;

        target_pub_->publish(target);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "TARGET x=%.3f z=%.3f",
            target.pose.position.x,
            target.pose.position.z);

        double error_x =
            world_error_.pose.position.x;

        double error_y =
            world_error_.pose.position.y;

        double error_z =
            world_error_.pose.position.z;

        double world_error_norm =
            std::sqrt(
                error_x * error_x +
                error_y * error_y +
                error_z * error_z);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "segment=%zu progress=%.2f error=%.4f",
            current_index_,
            progress_,
            world_error_norm);

        /* =====================================
           Progress segment continuously
           ===================================== */
        constexpr double progress_speed = 0.010;

        if (world_error_norm < 0.02)
        {
            progress_ = 1.0;
            /* progress_ += progress_speed;

            if (progress_ >= 1.0)
            {
                current_index_++;

                segment_initialized_ = false;

                RCLCPP_INFO(
                    get_logger(),
                    "Advanced to segment %zu",
                    current_index_);
            } */
        }
    }

private:
    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr world_pose_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped current_world_pose_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr
        error_sub_;

    geometry_msgs::msg::PoseStamped
        world_error_;

    bool error_received_ = false;

    std::vector<GCodePoint> points_;

    size_t current_index_ = 0;

    GCodePoint segment_start_;
    GCodePoint segment_end_;

        double progress_ = 0.0;
    bool job_complete_ = false;
    bool segment_initialized_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<
            GcodeWorldExecutor>());

    rclcpp::shutdown();

    return 0;
}

// end of file: ros2_ws/src/wbp_mobile_fabrication/src/gcode_world_executor.cpp