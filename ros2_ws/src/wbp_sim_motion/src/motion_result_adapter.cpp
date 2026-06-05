// start of file: ros2_ws/src/wbp_sim_motion/src/motion_result_adapter.cpp
#include <rclcpp/rclcpp.hpp>

#include <wbp_interfaces/msg/motion_runtime_state.hpp>
#include <wbp_sim_interfaces/msg/motion_result.hpp>

#include "wbp_interfaces/msg/segment_status.hpp"
#include <std_msgs/msg/empty.hpp>
#include "wbp_sim_interfaces/msg/motion_status.hpp"

class MotionResultAdapter : public rclcpp::Node
{
public:
    MotionResultAdapter() : Node("motion_result_adapter")
    {
        runtime_sub_ =
            this->create_subscription<
                wbp_interfaces::msg::MotionRuntimeState>(
                "/motion_runtime_state",
                10,
                std::bind(&MotionResultAdapter::runtimeCallback,
                          this,
                          std::placeholders::_1));

        result_pub_ =
            this->create_publisher<
                wbp_sim_interfaces::msg::MotionResult>(
                "/sim/motion/result",
                10);

        status_pub_ =
            this->create_publisher<
                wbp_sim_interfaces::msg::MotionStatus>(
                "/sim/motion/status",
                10);

        segment_status_sub_ =
            this->create_subscription<
                wbp_interfaces::msg::SegmentStatus>(
                "/segment_status",
                10,
                std::bind(
                    &MotionResultAdapter::on_segment_status,
                    this,
                    std::placeholders::_1));

        abort_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/motion_abort",
            10,
            [this](const std_msgs::msg::Empty::SharedPtr)
            {
                abort_active_ = true;
                RCLCPP_WARN(this->get_logger(), "Adapter: ABORT received");
            });

        RCLCPP_INFO(this->get_logger(),
                    "Motion Result Adapter started");
    }

private:
    /* void runtimeCallback(
        const wbp_interfaces::msg::MotionRuntimeState::SharedPtr msg)
    {
        uint32_t depth = msg->queue_depth;



        if (last_queue_depth_ != -1 && depth < last_queue_depth_)
        {
            publishSegmentComplete();
        }

        last_queue_depth_ = depth;



        if (msg->exec_state == EXEC_IDLE &&
            depth == 0 &&
            !program_complete_published_)
        {
            publishProgramComplete();
            program_complete_published_ = true;
        }
    } */

    void runtimeCallback(
        const wbp_interfaces::msg::MotionRuntimeState::SharedPtr msg)
    {
        if (msg->exec_state == EXEC_IDLE &&
            msg->queue_depth == 0 &&
            !program_complete_published_)
        {
            if (abort_active_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Program completion ignored due to abort");
                return;
            }

            wbp_sim_interfaces::msg::MotionResult result;

            result.sequence_id = 0; // program-level event
            result.status =
                wbp_sim_interfaces::msg::MotionResult::COMPLETED;

            result_pub_->publish(result);

            RCLCPP_INFO(this->get_logger(),
                        "Motion program complete");

            program_complete_published_ = true;
        }
    }

    /* void publishSegmentComplete()
    {
        if (abort_active_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Segment completion ignored due to active abort");
            return;
        }

        wbp_sim_interfaces::msg::MotionResult result;

        result.sequence_id = sequence_counter_++;
        result.status =
            wbp_sim_interfaces::msg::MotionResult::COMPLETED;

        result_pub_->publish(result);

        RCLCPP_INFO(this->get_logger(),
                    "Segment completed");
    }

    void publishProgramComplete()
    {
        if (abort_active_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Program completion ignored due to active abort");
            return;
        }
        wbp_sim_interfaces::msg::MotionResult result;

        result.sequence_id = sequence_counter_;
        result.status =
            wbp_sim_interfaces::msg::MotionResult::COMPLETED;

        result_pub_->publish(result);

        RCLCPP_INFO(this->get_logger(),
                    "Motion program complete");
    } */

    /* void on_segment_status(
        const wbp_interfaces::msg::SegmentStatus::SharedPtr msg)
    {
        wbp_sim_interfaces::msg::MotionStatus status;

        status.executing_segment = msg->segment_id;

        // SegmentStatus does not provide queue telemetry
        status.queue_depth = 0;
        status.queue_capacity = 0;
        status.ready = true;

        status_pub_->publish(status);
    } */

    void on_segment_status(
        const wbp_interfaces::msg::SegmentStatus::SharedPtr msg)
    {
        // Forward execution status to sim status (unchanged)
        wbp_sim_interfaces::msg::MotionStatus status;

        status.executing_segment = msg->segment_id;
        status.queue_depth = 0;
        status.queue_capacity = 0;
        status.ready = true;

        status_pub_->publish(status);

        // 🔥 NEW: detect completion directly
        if (msg->status == 2) // COMPLETED
        {
            if (abort_active_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Completion ignored due to abort");
                return;
            }

            wbp_sim_interfaces::msg::MotionResult result;

            result.sequence_id = msg->segment_id; // use real id
            result.status =
                wbp_sim_interfaces::msg::MotionResult::COMPLETED;

            result_pub_->publish(result);

            RCLCPP_INFO(this->get_logger(),
                        "Adapter: Segment %u completed",
                        msg->segment_id);
        }
    }

    rclcpp::Subscription<
        wbp_interfaces::msg::MotionRuntimeState>::SharedPtr runtime_sub_;

    rclcpp::Publisher<
        wbp_sim_interfaces::msg::MotionResult>::SharedPtr result_pub_;

    rclcpp::Subscription<wbp_interfaces::msg::SegmentStatus>::SharedPtr segment_status_sub_;

    rclcpp::Publisher<wbp_sim_interfaces::msg::MotionStatus>::SharedPtr status_pub_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr abort_sub_;

    bool abort_active_ = false;

    /* int last_queue_depth_ = -1;

    uint32_t sequence_counter_ = 1; */

    bool program_complete_published_ = false;

    static constexpr uint8_t EXEC_IDLE = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<MotionResultAdapter>());

    rclcpp::shutdown();

    return 0;
}

// end of file: ros2_ws/src/wbp_sim_motion/src/motion_result_adapter.cpp