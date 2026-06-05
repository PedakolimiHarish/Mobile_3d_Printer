// start of file: ros2_ws/src/wbp_firmware_motion_executor/src/motion_executor.cpp
#include "wbp_firmware_motion_executor/motion_executor.hpp"
#include <unordered_map>
#include <cmath>
#include <deque>
#include <algorithm>

static constexpr size_t SETTLE_TICKS_REQUIRED = 20;
static constexpr double VELOCITY_TOLERANCE = 0.001;

MotionExecutor::MotionExecutor()
    : Node("wbp_firmware_motion_executor"),
      exec_state_(ExecutionState::IDLE)
{
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    segment_sub_ = this->create_subscription<
        wbp_interfaces::msg::MotionSegment>(
        "/motion_segment",
        rclcpp::QoS(1000).reliable(),
        std::bind(&MotionExecutor::segmentCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<
        sensor_msgs::msg::JointState>(
        "/joint_states",
        10,
        std::bind(&MotionExecutor::jointStateCallback, this, std::placeholders::_1));

    actuator_pub_ = this->create_publisher<
        wbp_interfaces::msg::ActuatorCommand>("/actuator_command", 10);

    status_pub_ =
        this->create_publisher<wbp_interfaces::msg::SegmentStatus>(
            "/segment_status",
            10);

    control_timer_ = this->create_timer(
        std::chrono::milliseconds(2),
        std::bind(&MotionExecutor::controlLoop, this));

    control_period_ = 0.002;

    // 🔴 CRITICAL FIX — initialize feedrate override
    feedrate_override_cmd_ = 1.0;
    feedrate_override_actual_ = 1.0;

    reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/motion_reset",
        10,
        std::bind(&MotionExecutor::resetCallback, this, std::placeholders::_1));

    override_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/motion_override",
        10,
        std::bind(&MotionExecutor::overrideCallback, this, std::placeholders::_1));

    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/machine_estop",
        10,
        std::bind(&MotionExecutor::estopCallback, this, std::placeholders::_1));

    runtime_pub_ = this->create_publisher<
        wbp_interfaces::msg::MotionRuntimeState>(
        "/motion_runtime_state", 10);

    abort_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/motion_abort",
        10,
        std::bind(&MotionExecutor::abortCallback, this, std::placeholders::_1));

    initializeAxisConfiguration();
}

void MotionExecutor::segmentCallback(const wbp_interfaces::msg::MotionSegment::SharedPtr msg)
{

    // 🔴 [DBG-2] MACHINE STATE BLOCK
    if (machine_state_ != MachineState::NORMAL ||
        exec_state_ == ExecutionState::ABORTING)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "[DBG-2] REJECT: machine_state=%d exec_state=%d",
                     (int)machine_state_,
                     (int)exec_state_);

        publishStatus(4, msg->segment_id,
                      "Rejected — machine not in NORMAL state");
        return;
    }

    // 🔴 [DBG-3] JOINT STATE BLOCK
    if (!joint_state_ready_)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "[DBG-3] REJECT: joint_state_ready_=false");

        publishStatus(4, msg->segment_id, "Joint state not ready");
        return;
    }

    // 🔴 [DBG-4] QUEUE FULL BLOCK
    if (segment_queue_.size() >= MAX_QUEUE_SIZE)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "[DBG-4] REJECT: queue full size=%zu",
                     segment_queue_.size());

        publishStatus(4, msg->segment_id, "Queue full");
        return;
    }

    QueuedSegment seg;
    seg.segment_id = msg->segment_id;
    seg.joint_names = msg->joint_names;
    seg.q_target = msg->target_positions;

    seg.max_velocity = msg->max_velocity;
    seg.max_acceleration = msg->max_acceleration;
    seg.position_tolerance = msg->position_tolerance;
    seg.extrusion_enabled = msg->extrusion_enabled;

    seg.v_start = msg->v_start;
    seg.v_end = msg->v_end;
    seg.v_max = msg->v_max;

    segment_queue_.push_back(seg);

    publishStatus(0, seg.segment_id, "Segment queued");

    if (exec_state_ == ExecutionState::IDLE &&
        !segment_queue_.empty() &&
        machine_state_ == MachineState::NORMAL)
    {
        // 🔴 [DBG-8] START TRIGGER
        RCLCPP_ERROR(this->get_logger(),
                     "[DBG-8] STARTING NEXT SEGMENT");

        startNextSegment();
    }
}

void MotionExecutor::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    joint_state_ready_ = true;

    last_joint_state_map_.clear();
    last_joint_velocity_map_.clear(); // 🔥 ADD THIS

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        last_joint_state_map_[msg->name[i]] = msg->position[i];

        if (i < msg->velocity.size())
            last_joint_velocity_map_[msg->name[i]] = msg->velocity[i];
        else
            last_joint_velocity_map_[msg->name[i]] = 0.0;
    }

    if (!joint_names_.empty())
    {
        q_current_.resize(joint_names_.size());

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            const auto &name = joint_names_[i];

            if (last_joint_state_map_.count(name))
                q_current_[i] = last_joint_state_map_[name];
        }
    }
}

void MotionExecutor::publishStatus(uint8_t status,
                                   uint32_t segment_id,
                                   const std::string &reason)
{
    wbp_interfaces::msg::SegmentStatus msg;

    msg.segment_id = segment_id;
    msg.status = status;

    status_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
                "[EXEC STATUS] seg=%u status=%u (%s)",
                segment_id, status, reason.c_str());
}

void MotionExecutor::controlLoop()
{
    if (exec_state_ != ExecutionState::RUNNING)
        return;

    if (joint_names_.empty())
        return;

    double dt = control_period_;

    std::vector<double> commanded = q_current_;

    bool reached = true;

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        if (!isStructuralAxis(joint_names_[i]))
            continue;

        double error = q_target_[i] - q_current_[i];

        double step = max_velocity_ * dt;

        if (std::abs(error) > step)
        {
            commanded[i] = q_current_[i] + (error > 0 ? step : -step);
            reached = false;
        }
        else
        {
            commanded[i] = q_target_[i];
        }
    }

    // 🔴 Update internal state
    q_current_ = commanded;

    // 🔴 Publish motion
    wbp_interfaces::msg::ActuatorCommand cmd;
    cmd.joint_names = joint_names_;
    cmd.commanded_positions = commanded;

    actuator_pub_->publish(cmd);

    // 🔴 Completion
    if (reached)
    {
        publishStatus(2, active_segment_id_, "Segment complete");

        if (!segment_queue_.empty())
        {
            startNextSegment();
        }
        else
        {
            exec_state_ = ExecutionState::IDLE;
        }
    }
}

void MotionExecutor::startNextSegment()
{
    reversal_reached_zero_ = false;
    settle_counter_ = 0;
    if (segment_queue_.empty())
    {
        exec_state_ = ExecutionState::IDLE;
        return;
    }

    starvation_logged_ = false;
    current_segment_ = segment_queue_.front();
    segment_queue_.pop_front();
    allow_continuous_motion_ =
        segment_queue_.size() >= lookahead_depth_;

    active_segment_id_ = current_segment_.segment_id;
    joint_names_ = current_segment_.joint_names;
    q_target_ = current_segment_.q_target; // 🔥 MOVE THIS UP

    // Segment requests motion
    max_velocity_ = current_segment_.v_max;
    max_acceleration_ = current_segment_.max_acceleration;

    // 🔴 FIX: initialize jerk limit
    max_jerk_ = std::numeric_limits<double>::max();

    for (const auto &name : joint_names_)
    {
        auto it = axis_config_map_.find(name);
        if (it == axis_config_map_.end())
            continue;

        const auto &cfg = it->second;

        max_jerk_ = std::min(max_jerk_, cfg.max_jerk);
    }

    RCLCPP_ERROR(this->get_logger(),
                 "[JERK INIT] max_jerk_=%.6f",
                 max_jerk_);

    // Clamp by machine authority
    for (const auto &name : joint_names_)
    {
        auto it = axis_config_map_.find(name);
        if (it == axis_config_map_.end())
            continue;

        const auto &cfg = it->second;

        max_velocity_ = std::min(max_velocity_, cfg.max_velocity);
        max_acceleration_ = std::min(max_acceleration_, cfg.max_acceleration);
    }

    if (max_velocity_ <= 1e-6)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "INVALID max_velocity_ = %.6f → forcing minimum",
                     max_velocity_);

        max_velocity_ = 0.1; // safe fallback
    }

    position_tolerance_ = current_segment_.position_tolerance;

    q_start_.resize(joint_names_.size());
    q_current_.resize(joint_names_.size());

    // =====================================
    // Axis Authority Check
    // =====================================

    for (const auto &name : joint_names_)
    {
        auto &axis_rt = axis_runtime_map_[name];

        if (axis_rt.state == AxisState::DISABLED ||
            axis_rt.state == AxisState::FAULT ||
            axis_rt.state == AxisState::ESTOP)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Axis %s not authorized for motion",
                         name.c_str());

            exec_state_ = ExecutionState::FAULT;

            publishStatus(4, active_segment_id_,
                          "Axis not authorized");

            return;
        }
    }

    // ------------------------------------------------------
    // Determine segment start position
    // ------------------------------------------------------

    if (first_segment_)
    {
        // First segment → use measured position
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            const auto &name = joint_names_[i];

            if (last_joint_state_map_.count(name))
                q_start_[i] = last_joint_state_map_[name];
            else
                q_start_[i] = 0.0;
        }

        first_segment_ = false;
    }
    else
    {
        // Subsequent segments → use previous commanded target
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            q_start_[i] = q_target_previous_[i];
        }
    }

    q_current_ = q_start_;

    v_start_.assign(joint_names_.size(), structural_velocity_);
    v_end_.assign(joint_names_.size(), current_segment_.v_end);

    max_velocity_ = current_segment_.v_max;

    // ======================================================
    // 🔴 DEBUG: verify velocity propagation
    // ======================================================

    if (!v_start_.empty())
    {
        RCLCPP_WARN(this->get_logger(),
                    "[VEL PROP] seg=%u v_start=%.6f",
                    active_segment_id_,
                    v_start_[0]);
    }

    // ======================================================
    // 🔴 DEBUG: blending result
    // ======================================================

    if (!v_end_.empty())
    {
        RCLCPP_WARN(this->get_logger(),
                    "[VEL BLEND] seg=%u planned_v_end=%.6f",
                    active_segment_id_,
                    v_end_[0]);
    }

    // ---------------------------------------
    // Soft Limit Enforcement
    // ---------------------------------------

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        auto it = axis_config_map_.find(joint_names_[i]);
        if (it == axis_config_map_.end())
            continue;

        const auto &cfg = it->second;
        double target = q_target_[i];

        if (target < cfg.min_position || target > cfg.max_position)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Soft limit violation on %s",
                         joint_names_[i].c_str());

            auto &axis_rt = axis_runtime_map_[joint_names_[i]];
            axis_rt.hard_limit_triggered = true;
            axis_rt.state = AxisState::FAULT;

            machine_state_ = MachineState::FAULT_LATCHED;
            exec_state_ = ExecutionState::FAULT;

            publishStatus(4, active_segment_id_,
                          "Soft limit violation (latched)");

            return;
        }
    }

    // ------------------------------------------------------
    // Compute Structural Path Length
    // ------------------------------------------------------

    structural_target_distance_ = 0.0;

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        if (isStructuralAxis(joint_names_[i]))
        {
            double dq = q_target_[i] - q_start_[i];
            structural_target_distance_ += dq * dq;
        }
    }

    structural_target_distance_ = std::sqrt(structural_target_distance_);

    // ============================================================
    // 🔥 CRITICAL FIX — Prevent zero-velocity dead segment
    // ============================================================

    bool zero_velocity_segment = true;

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        if (!isStructuralAxis(joint_names_[i]))
            continue;

        if (std::abs(v_start_[i]) > 1e-6 || std::abs(v_end_[i]) > 1e-6)
        {
            zero_velocity_segment = false;
            break;
        }
    }

    // If segment has distance but zero velocity → INVALID → FIX
    if (structural_target_distance_ > 1e-6 && zero_velocity_segment)
    {
        const double MIN_VEL = 0.05; // tune if needed

        RCLCPP_WARN(this->get_logger(),
                    "FIX: zero-velocity segment detected → injecting minimum velocity");

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            if (!isStructuralAxis(joint_names_[i]))
                continue;

            v_start_[i] = MIN_VEL;
            v_end_[i] = MIN_VEL;
        }

        v_end_structural_ = MIN_VEL;
    }

    // Estimate theoretical segment duration
    if (max_velocity_ > 1e-6)
        segment_duration_ = structural_target_distance_ / max_velocity_;
    else
        segment_duration_ = 0.0;

    // Add acceleration time estimate (conservative)
    segment_duration_ += max_velocity_ / max_acceleration_;

    segment_start_time_ = global_time_;

    RCLCPP_WARN(this->get_logger(),
                "[MOTION DBG] s=%.6f rem=%.6f v=%.6f a=%.6f",
                arc_length_s_,
                structural_target_distance_ - arc_length_s_,
                structural_velocity_,
                structural_acceleration_);

    RCLCPP_INFO(this->get_logger(),
                "START segment %u | queue size=%zu | distance=%.6f | carry_v=%.6f",
                active_segment_id_,
                segment_queue_.size(),
                structural_target_distance_,
                structural_velocity_);

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Joint %s | start=%.6f target=%.6f structural=%d",
                    joint_names_[i].c_str(),
                    q_start_[i],
                    q_target_[i],
                    isStructuralAxis(joint_names_[i]));
    }

    // Store target for next segment continuity
    q_target_previous_ = q_target_;

    for (const auto &name : joint_names_)
    {
        axis_runtime_map_[name].state = AxisState::ARMED;
    }

    v_end_structural_ = 0.0;

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        if (!isStructuralAxis(joint_names_[i]))
            continue;

        double v = std::abs(v_end_[i]);

        if (v > v_end_structural_)
            v_end_structural_ = v;
    }

    // 🔴 FIX 5: final segment must stop
    if (segment_queue_.empty())
    {
        v_end_structural_ = 0.0;
    }

    // ======================================================
    // ✅ FIX: preserve real physical velocity across segments
    // ======================================================

    // DO NOT overwrite structural_velocity_
    // It already contains real runtime velocity

    RCLCPP_WARN(this->get_logger(),
                "[VEL CONTINUITY] seg=%u carry_v=%.6f",
                active_segment_id_,
                structural_velocity_);

    // 🔥 FIX 4 — Kickstart velocity if stuck at zero
    if (structural_velocity_ < 1e-6)
    {
        RCLCPP_WARN(this->get_logger(),
                    "[KICKSTART] injecting initial velocity");

        structural_velocity_ = 0.1; // small non-zero start
        structural_acceleration_ = max_acceleration_;
    }

    // Reset acceleration for clean transition
    // structural_acceleration_ = 0.0;

    exec_state_ = ExecutionState::RUNNING;

    publishStatus(1, active_segment_id_, "Segment executing");
}

void MotionExecutor::resetCallback(const std_msgs::msg::Empty::SharedPtr)
{
    if (machine_state_ == MachineState::ESTOP)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Cannot reset while E-STOP active");
        return;
    }

    RCLCPP_WARN(this->get_logger(),
                "RESET — clearing faults");

    for (auto &pair : axis_runtime_map_)
    {
        pair.second.hard_limit_triggered = false;
        pair.second.state = AxisState::IDLE;
    }

    machine_state_ = MachineState::NORMAL;

    segment_queue_.clear();

    structural_velocity_ = 0.0;
    structural_acceleration_ = 0.0;
    // arc_length_s_ = 0.0;

    allow_continuous_motion_ = false;
    first_segment_ = true;
    active_segment_id_ = 0;

    exec_state_ = ExecutionState::IDLE;

    publishStatus(3, 0, "Controller reset");
}

bool MotionExecutor::isStructuralAxis(const std::string &name)
{
    auto it = axis_config_map_.find(name);
    if (it == axis_config_map_.end())
        return false;

    return (it->second.type == AxisType::STRUCTURAL_LINEAR ||
            it->second.type == AxisType::STRUCTURAL_ROTARY);
}

void MotionExecutor::overrideCallback(
    const std_msgs::msg::Float64::SharedPtr msg)
{
    double value = msg->data;

    // Safety clamp
    if (value < 0.0)
        value = 0.0;

    if (value > 2.0)
        value = 2.0;

    feedrate_override_cmd_ = value;

    RCLCPP_DEBUG(this->get_logger(),
                 "Feedrate override command set to %.2f",
                 feedrate_override_cmd_);
}

void MotionExecutor::initializeAxisConfiguration()
{
    AxisConfig base;
    base.name = "base_to_mast_joint";
    base.type = AxisType::STRUCTURAL_LINEAR;
    base.max_velocity = 0.5;
    base.max_acceleration = 1.0;
    base.max_jerk = 50.0;
    base.min_position = 0.0;
    base.max_position = 2.0;
    base.enabled = true;

    axis_config_map_[base.name] = base;

    AxisConfig carriage = base;
    carriage.name = "carriage_to_tool_joint";
    axis_config_map_[carriage.name] = carriage;

    AxisConfig mast = base;
    mast.name = "mast_to_carriage_joint";
    axis_config_map_[mast.name] = mast;

    AxisConfig extruder;
    extruder.name = "tool_mount_joint";
    extruder.type = AxisType::TOOL_EXTRUDER;
    extruder.max_velocity = 10.0;
    extruder.max_acceleration = 50.0;
    extruder.max_jerk = 100.0;
    extruder.min_position = -1e9;
    extruder.max_position = 1e9;
    extruder.enabled = true;

    axis_config_map_[extruder.name] = extruder;

    AxisRuntime runtime;
    runtime.state = AxisState::IDLE;
    axis_runtime_map_[base.name] = runtime;
    axis_runtime_map_[carriage.name] = runtime;
    axis_runtime_map_[mast.name] = runtime;
    axis_runtime_map_[extruder.name] = runtime;
}

void MotionExecutor::estopCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        if (machine_state_ != MachineState::ESTOP)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "EMERGENCY STOP ACTIVATED");

            machine_state_ = MachineState::ESTOP;
            exec_state_ = ExecutionState::FAULT;

            structural_velocity_ = 0.0;
            structural_acceleration_ = 0.0;

            segment_queue_.clear();

            publishStatus(4, active_segment_id_,
                          "Emergency stop");
        }
    }
    else
    {
        if (machine_state_ == MachineState::ESTOP)
        {
            RCLCPP_WARN(this->get_logger(),
                        "EMERGENCY STOP RELEASED — waiting for RESET");

            machine_state_ = MachineState::NORMAL;
            // Still FAULT until reset
        }
    }
}

void MotionExecutor::abortCallback(
    const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg; // unused

    abort_requested_ = true;

    if (!abort_logged_)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "ABORT requested — entering controlled stop");
        abort_logged_ = true;
    }

    // 🔥 CRITICAL: switch state
    exec_state_ = ExecutionState::ABORTING;

    // 🔥 CRITICAL: stop future motion
    segment_queue_.clear();
}

// end of motion_executor.cpp