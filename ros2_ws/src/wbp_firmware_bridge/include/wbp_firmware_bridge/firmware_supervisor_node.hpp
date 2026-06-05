#pragma once

#include <rclcpp/rclcpp.hpp>
#include <wbp_interfaces/msg/machine_state.hpp>
#include <wbp_interfaces/srv/lifecycle_command.hpp>
#include "wbp_interfaces/srv/job_submit.hpp"

#include "ipc_state_reader.hpp"
#include "ipc_command_writer.hpp"

namespace wbp_firmware_bridge
{

    class FirmwareSupervisorNode : public rclcpp::Node
    {
    public:
        FirmwareSupervisorNode();

    private:
        void publish_state();
        void handle_command(
            const std::shared_ptr<wbp_interfaces::srv::LifecycleCommand::Request> req,
            std::shared_ptr<wbp_interfaces::srv::LifecycleCommand::Response> res);

        IpcStateReader state_reader_;
        IpcCommandWriter command_writer_;

        rclcpp::Service<wbp_interfaces::srv::JobSubmit>::SharedPtr job_submit_srv_;
        rclcpp::Publisher<wbp_interfaces::msg::MachineState>::SharedPtr state_pub_;
        rclcpp::Service<wbp_interfaces::srv::LifecycleCommand>::SharedPtr command_srv_;
        rclcpp::TimerBase::SharedPtr timer_;
        bool state_connected_{false};
    };

} // namespace wbp_firmware_bridge
