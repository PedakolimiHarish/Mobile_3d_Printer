#include "wbp_firmware_motion_executor/motion_monitor.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionMonitor>());
    rclcpp::shutdown();
    return 0;
}