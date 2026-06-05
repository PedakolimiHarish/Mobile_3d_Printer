#include "wbp_firmware_motion_executor/motion_executor.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionExecutor>());
    rclcpp::shutdown();
    return 0;
}
