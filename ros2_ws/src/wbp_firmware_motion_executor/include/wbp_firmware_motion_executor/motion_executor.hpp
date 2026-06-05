// start of file: ros2_ws/src/wbp_firmware_motion_executor/include/wbp_firmware_motion_executor/motion_executor.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include <wbp_interfaces/msg/motion_segment.hpp>
#include <wbp_interfaces/msg/actuator_command.hpp>
#include <wbp_interfaces/msg/segment_status.hpp>
#include "wbp_interfaces/msg/motion_runtime_state.hpp"
#include <vector>
#include <string>
#include <unordered_map>

class MotionExecutor : public rclcpp::Node
{
public:
    MotionExecutor();

private:
    // ===== ROS Interfaces =====
    rclcpp::Subscription<wbp_interfaces::msg::MotionSegment>::SharedPtr segment_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    rclcpp::Publisher<wbp_interfaces::msg::ActuatorCommand>::SharedPtr actuator_pub_;
    rclcpp::Publisher<wbp_interfaces::msg::SegmentStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<wbp_interfaces::msg::MotionRuntimeState>::SharedPtr runtime_pub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr abort_sub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    double motion_override_ = 1.0;
    bool abort_requested_ = false;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr override_sub_;

    // ===== Execution State =====
    enum class ExecutionState
    {
        IDLE = 0,
        RUNNING = 1,
        PAUSED = 2,
        ABORTING = 3,
        FAULT = 4,
        STARVATION_STOP = 5
    };

    struct QueuedSegment
    {
        uint32_t segment_id;
        std::vector<std::string> joint_names;
        std::vector<double> q_target;
        double max_velocity;
        double max_acceleration;
        double position_tolerance;

        bool extrusion_enabled = false;

        double v_start;
        double v_end;
        double v_max;
    };

    enum class AxisType
    {
        STRUCTURAL_LINEAR,
        STRUCTURAL_ROTARY,
        TOOL_EXTRUDER,
        TOOL_ROTARY,
        AUXILIARY
    };

    struct AxisConfig
    {
        std::string name;
        AxisType type;

        // Motion limits (machine authority)
        double max_velocity;
        double max_acceleration;
        double max_jerk;

        // Travel limits (soft limits)
        double min_position;
        double max_position;

        // Enable flags
        bool enabled;
    };

    enum class MachineState
    {
        NORMAL,
        ESTOP,
        FAULT_LATCHED
    };

    MachineState machine_state_ = MachineState::NORMAL;

    enum class AxisState
    {
        DISABLED,
        IDLE,
        ARMED,
        FAULT,
        ESTOP
    };

    struct AxisRuntime
    {
        AxisState state = AxisState::DISABLED;
        bool hard_limit_triggered = false;
        bool soft_limit_triggered = false;
        bool drive_fault = false;
    };

    std::unordered_map<std::string, AxisRuntime> axis_runtime_map_;

    // Represents full motion profile for ONE joint during a segment
    struct JointProfile
    {
        double t1; // Acceleration phase duration
        double t2; // Constant velocity (cruise) phase duration
        double t3; // Deceleration phase duration

        double v_peak; // Peak velocity reached during this segment

        double total_time; // Total duration = t1 + t2 + t3
    };

    std::vector<JointProfile> joint_profiles_;

    double solvePeakVelocity(
        double D,
        double v0,
        double vf,
        double a,
        double T_target);

    ExecutionState exec_state_;
    std::deque<QueuedSegment> segment_queue_;
    QueuedSegment current_segment_;

    struct EnvelopeSegment
    {
        double distance; // absolute distance for structural axis
        double v_entry;  // entry velocity
        double v_exit;   // exit velocity
    };

    std::unordered_map<std::string, AxisConfig> axis_config_map_;

    // ===== Internal State =====
    uint32_t active_segment_id_;

    std::vector<std::string> joint_names_;
    std::vector<double> q_start_;
    std::vector<double> q_target_;
    std::vector<double> q_current_;

    std::vector<double> v_start_;
    std::vector<double> v_end_;
    std::vector<double> v_current_;

    double max_velocity_;
    double max_acceleration_;
    double position_tolerance_;

    double segment_duration_;
    double feedrate_override_cmd_ = 1.0;
    double feedrate_override_actual_ = 1.0;

    // Units: override change per second
    double override_ramp_rate_ = 1.50;

    rclcpp::Time in_position_start_time_;
    bool in_position_active_ = false;

    double settle_time_required_ = 0.1; // 100 ms
    double timeout_factor_ = 2.0;       // 2x theoretical duration
    double internal_time_ = 0.0;
    double control_period_ = 0.002; // 2 ms
    size_t lookahead_depth_ = 5;
    double extrusion_gain_ = 1.0;
    bool abort_logged_ = false;
    double global_time_ = 0.0;
    bool starvation_logged_ = false;
    double structural_velocity_ = 0.0; // scalar velocity magnitude

    double structural_target_distance_ = 0.0;

    double structural_acceleration_ = 0.0;
    double max_jerk_ = 10.0; // tune later

    double segment_start_time_ = 0.0;
    double segment_timeout_ = 0.0;

    double v_end_structural_ = 0.0;
    double arc_length_s_ = 0.0;

    bool joint_state_ready_ = false;
    bool allow_continuous_motion_ = false;
    bool reversal_reached_zero_ = false;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;

    std::vector<double> q_target_previous_;
    bool first_segment_ = true;

    bool reversal_reported_ = false;

    void segmentCallback(const wbp_interfaces::msg::MotionSegment::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    void controlLoop();
    void startNextSegment();
    void computeBlendingWithNext();

    static constexpr size_t MAX_QUEUE_SIZE = 10;

    void overrideCallback(const std_msgs::msg::Float64::SharedPtr msg);

    double computeSynchronizedDuration();
    double computePositionAtTime(size_t joint_index, double t);

    void publishStatus(uint8_t status_code,
                       uint32_t segment_id,
                       const std::string &message);
    void initializeAxisConfiguration();

    bool withinTolerance();
    bool isDirectionReversal();

    bool isStructuralAxis(const std::string &name);
    double computeVelocityAtTime(size_t i, double t);
    void abortCallback(const std_msgs::msg::Empty::SharedPtr msg);

    std::unordered_map<std::string, double> last_joint_state_map_;
    std::unordered_map<std::string, double> last_joint_velocity_map_;
    std::vector<double> last_v_end_;
    size_t settle_counter_ = 0;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
    void resetCallback(const std_msgs::msg::Empty::SharedPtr msg);
    void estopCallback(const std_msgs::msg::Bool::SharedPtr msg);
};

// end of file