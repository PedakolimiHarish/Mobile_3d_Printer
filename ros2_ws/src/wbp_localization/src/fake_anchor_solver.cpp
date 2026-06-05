// start of file: ros2_ws/src/wbp_localization/src/fake_anchor_solver.cpp
#include <cmath>
#include <random>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include "wbp_interfaces/msg/anchor_correction.hpp"

class FakeAnchorSolver : public rclcpp::Node
{
public:
    FakeAnchorSolver()
        : Node("fake_anchor_solver")
    {

        odom_sub_ =
            create_subscription<
                nav_msgs::msg::Odometry>(
                "/odom",
                50,
                std::bind(
                    &FakeAnchorSolver::odom_callback,
                    this,
                    std::placeholders::_1));

        correction_pub_ =
            create_publisher<
                wbp_interfaces::msg::AnchorCorrection>(
                "/anchor_correction",
                10);

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &FakeAnchorSolver::publish_correction,
                    this));

        RCLCPP_INFO(
            get_logger(),
            "FakeAnchorSolver started");
    }

private:
    void odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_x_ =
            msg->pose.pose.position.x;

        odom_y_ =
            msg->pose.pose.position.y;

        double qw =
            msg->pose.pose.orientation.w;

        double qz =
            msg->pose.pose.orientation.z;

        double qx =
            msg->pose.pose.orientation.x;

        double qy =
            msg->pose.pose.orientation.y;

        double siny_cosp =
            2.0 * (qw * qz + qx * qy);

        double cosy_cosp =
            1.0 - 2.0 * (qy * qy + qz * qz);

        odom_theta_ =
            std::atan2(
                siny_cosp,
                cosy_cosp);

        odom_received_ = true;
    }

    void publish_correction()
    {
        if (!odom_received_)
        {
            return;
        }

        // =====================================
        // SIMULATED ODOM DRIFT
        // =====================================

        simulated_drift_x_ += 0.0005;

        // =====================================
        // FAKE ANCHOR LOCALIZATION
        // =====================================

        double localized_x =
            odom_x_ - simulated_drift_x_;

        double localized_y =
            odom_y_;

        double localized_theta =
            odom_theta_;

        // =====================================
        // COMPUTE MAP->ODOM CORRECTION
        // =====================================

        wbp_interfaces::msg::AnchorCorrection correction;

        correction.x =
            localized_x - odom_x_;

        correction.y =
            localized_y - odom_y_;

        correction.theta =
            localized_theta - odom_theta_;

        correction.valid = true;

        correction_pub_->publish(correction);

        /*  RCLCPP_INFO(
             get_logger(),
             "Anchor correction x=%.3f",
             correction.x); */
    }

private:
    rclcpp::Subscription<
        nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<
        wbp_interfaces::msg::AnchorCorrection>::SharedPtr correction_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool odom_received_ = false;

    double odom_x_ = 0.0;
    double odom_y_ = 0.0;
    double odom_theta_ = 0.0;

    double simulated_drift_x_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<FakeAnchorSolver>());

    rclcpp::shutdown();

    return 0;
}

// end of file: ros2_ws/src/wbp_localization/src/fake_anchor_solver.cpp