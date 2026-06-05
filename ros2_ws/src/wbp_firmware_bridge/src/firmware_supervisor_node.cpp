#include "wbp_firmware_bridge/firmware_supervisor_node.hpp"
namespace wbp_firmware_bridge
{

    FirmwareSupervisorNode::FirmwareSupervisorNode()
        : Node("firmware_supervisor")
    {
        /* state_pub_ = create_publisher<wbp_interfaces::msg::MachineState>(
            "/machine_state", 10); */

        command_srv_ = create_service<wbp_interfaces::srv::LifecycleCommand>(
            "/firmware/command_request",
            std::bind(&FirmwareSupervisorNode::handle_command, this,
                      std::placeholders::_1, std::placeholders::_2));
    }

    void FirmwareSupervisorNode::handle_command(
        const std::shared_ptr<wbp_interfaces::srv::LifecycleCommand::Request> req,
        std::shared_ptr<wbp_interfaces::srv::LifecycleCommand::Response> res)
    {
        if (!command_writer_.connect())
        {
            res->accepted = false;
            return;
        }

        wbp_command_ipc_t cmd{};
        cmd.command = static_cast<uint32_t>(req->command);

        res->accepted = command_writer_.send(cmd);
    }

} // namespace wbp_firmware_bridge

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wbp_firmware_bridge::FirmwareSupervisorNode>());
    rclcpp::shutdown();
    return 0;
}
