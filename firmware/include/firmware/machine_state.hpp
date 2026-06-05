// start of firmware/include/firmware/machine_state.hpp
#pragma once
#include <cstdint>
#include <array>
#include <cstddef>

namespace firmware
{

    enum class SystemState : uint8_t
    {
        BOOT,            // 0
        IDLE,            // 1
        READY,           // 2
        EXECUTING_PRINT, // 3
        PAUSED,          // 4
        RESUMING,        // 5
        ABORTING,        // 6
        FAULT,           // 7
        RECOVERING       // 🔒 Phase 29
    };

    enum class MotionMode : uint8_t
    {
        STOPPED,
        SAFE_TRAVEL
    };

    enum class JobState : uint8_t
    {
        NONE,
        ACTIVE,
        COMPLETED,
        ABORTED
    };

    enum class SafetyState : uint8_t
    {
        OK,
        FAULT
    };

    struct ArmPose
    {
        static constexpr std::size_t DOF = 6;
        std::array<double, DOF> joint{};
        bool kinematic_valid{false};
    };

    struct MotionState
    {
        ArmPose arm;
        MotionMode mode{MotionMode::STOPPED};
    };

    struct JobContext
    {
        uint32_t job_id{0};
        uint32_t layer{0};
        uint32_t segment{0};
        JobState state{JobState::NONE};
    };

    struct SafetyContext
    {
        SafetyState state{SafetyState::OK};
        uint32_t fault_code{0};
    };

    enum class CommandOutcome : uint8_t
    {
        NONE,
        ACCEPTED,
        REJECTED,
        FAULTED
    };

    enum class LocalizationMode : uint8_t
    {
        FREE = 0,    // No anchors, odometry-only
        ANCHORED = 1 // Anchors enforced
    };

    struct OperatorActions
    {
        bool can_submit_job;
        bool can_start;
        bool can_resume;
        bool can_clear_job;
        bool can_recover;
    };

    enum class CompletionReason : uint8_t
    {
        NONE = 0,
        NORMAL,
        STARVATION,
        ABORTED
    };

    struct MachineState
    {
        SystemState system = SystemState::IDLE;

        LocalizationMode localization_mode{LocalizationMode::FREE};
        bool anchors_valid{false};
        uint32_t boot_id{0};

        MotionState motion;
        JobContext job;
        SafetyContext safety;
        uint64_t monotonic_time_ms{0};

        bool job_present{false};
        uint64_t job_id{0};
        uint32_t job_schema_version{0};
        char job_hash[64]{};
        char last_job_rejection[128]{};

        uint32_t last_command_seq{0};
        CommandOutcome last_command_outcome{CommandOutcome::NONE};

        // Phase 18
        bool localization_valid{false};
        uint32_t active_anchor_id{0};
        double localization_confidence{0.0};

        bool resume_available = false; // 🔒 snapshot exists & valid
        bool resume_consumed = false;
        bool recovery_in_progress{false};

        bool operator_recovery_required;
        bool resume_permanently_disabled;
        OperatorActions operator_actions;

        // motion pipeline state (authoritative))
        bool motion_idle = true;

        bool ingress_completed = false;

        CompletionReason last_completion_reason{CompletionReason::NONE};
    };

} // namespace firmware

// end of firmware/include/firmware/machine_state.hpp