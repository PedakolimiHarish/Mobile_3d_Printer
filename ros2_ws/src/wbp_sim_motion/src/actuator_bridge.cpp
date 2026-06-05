// start of file: ros2_ws/src/wbp_sim_motion/src/actuator_bridge.cpp
#include <rclcpp/rclcpp.hpp>
#include <wbp_interfaces/msg/actuator_command.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ActuatorBridge : public rclcpp::Node
{
public:
    ActuatorBridge() : Node("wbp_actuator_bridge")
    {
        sub_ = create_subscription<wbp_interfaces::msg::ActuatorCommand>(
            "/actuator_command",
            10,
            std::bind(&ActuatorBridge::callback, this, std::placeholders::_1));

        pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_position_controller/commands",
            10);
    }

private:
    void callback(const wbp_interfaces::msg::ActuatorCommand::SharedPtr msg)
    {
        // Controller expected order
        std::vector<std::string> controller_order = {
            "base_to_mast_joint",
            "carriage_to_tool_joint",
            "mast_to_carriage_joint",
            "tool_mount_joint"};

        std_msgs::msg::Float64MultiArray out;
        out.data.resize(controller_order.size());

        for (size_t i = 0; i < controller_order.size(); ++i)
        {
            const auto &target_name = controller_order[i];

            for (size_t j = 0; j < msg->joint_names.size(); ++j)
            {
                if (msg->joint_names[j] == target_name)
                {
                    out.data[i] = msg->commanded_positions[j];
                    break;
                }
            }
        }

        pub_->publish(out);
    }

    rclcpp::Subscription<wbp_interfaces::msg::ActuatorCommand>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorBridge>());
    rclcpp::shutdown();
    return 0;
}

// end of actuator_bridge.cpp