// start of file: ros2_ws/src/wbp_program_manager/src/program_manager.cpp
#include <rclcpp/rclcpp.hpp>
#include <wbp_interfaces/msg/motion_segment.hpp>
// #include <wbp_interfaces/msg/motion_runtime_state.hpp>
#include "wbp_interfaces/ipc/job_ingress_ipc.h"
#include "wbp_interfaces/ipc/segment.hpp"
#include "wbp_interfaces/srv/start_job.hpp"
#include "wbp_interfaces/msg/machine_state.hpp"
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include "gcode_parser.cpp"

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <yaml-cpp/yaml.h>

class MotionLoader : public rclcpp::Node
{

    /* static constexpr size_t STREAM_WINDOW = 6;
    static constexpr size_t LOW_WATERMARK = 3; */

    /* size_t executor_queue_depth_ = 0;
    size_t executor_queue_capacity_ = 0; */

    wbp_job_ingress_ipc_t *job_ipc_ = nullptr;

    // bool ingress_signaled_ = false;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gcode_sub_;

public:
    MotionLoader() : Node("program_manager")
    {
        /* publisher_ = this->create_publisher<
            wbp_interfaces::msg::MotionSegment>("/motion_segment", 10); */

        yaml_file_ = this->declare_parameter("yaml_file", "");

        // 🔥 YAML is optional now (G-code pipeline)
        if (!yaml_file_.empty())
        {
            RCLCPP_WARN(this->get_logger(),
                        "[PM] YAML mode still enabled (not recommended)");
        }

        state_sub_ =
            this->create_subscription<wbp_interfaces::msg::MachineState>(
                "/machine_state",
                10,
                std::bind(&MotionLoader::stateCallback, this, std::placeholders::_1));

        /*  runtime_sub_ =
            this->create_subscription<wbp_interfaces::msg::MotionRuntimeState>(
                "/motion_runtime_state",
                10,
                std::bind(&MotionLoader::runtimeCallback, this, std::placeholders::_1));

        stream_timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(&MotionLoader::streamSegments, this));

        segment_pub_ = this->create_publisher<wbp_interfaces::msg::MotionSegment>(
            "/motion_segment", 100); */

        /* start_stream_sub_ =
            this->create_subscription<std_msgs::msg::Empty>(
                "/start_job_stream",
                10,
                [this](const std_msgs::msg::Empty::SharedPtr)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "[PM] START trigger received → submitting job");
                }); */

        gcode_sub_ =
            this->create_subscription<std_msgs::msg::String>(
                "/gcode_file",
                10,
                [this](const std_msgs::msg::String::SharedPtr msg)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "[PM] G-code file received: %s",
                                msg->data.c_str());

                    loadGcode(msg->data); // 🔥 NOW ACTIVE
                });

        RCLCPP_INFO(this->get_logger(),
                    "Program Manager ready. Waiting for START signal.");

        int fd = shm_open(WBP_JOB_SHM_NAME, O_RDWR, 0666);

        if (fd >= 0)
        {
            void *ptr = mmap(nullptr,
                             sizeof(wbp_job_ingress_ipc_t),
                             PROT_READ | PROT_WRITE,
                             MAP_SHARED,
                             fd,
                             0);

            if (ptr != MAP_FAILED)
            {
                job_ipc_ = static_cast<wbp_job_ingress_ipc_t *>(ptr);
            }
        }
    }

private:
    void loadProgram(const std::string &file)
    {
        YAML::Node config = YAML::LoadFile(file);

        auto segments = config["motion_segments"];

        for (auto seg : segments)
        {
            wbp_interfaces::msg::MotionSegment msg;

            msg.segment_id = seg["segment_id"].as<uint32_t>();
            msg.joint_names = seg["joint_names"].as<std::vector<std::string>>();
            msg.target_positions = seg["target_positions"].as<std::vector<double>>();
            msg.max_velocity = seg["max_velocity"].as<double>();
            msg.max_acceleration = seg["max_acceleration"].as<double>();
            msg.position_tolerance = seg["position_tolerance"].as<double>();

            program_segments_.push_back(msg);

            RCLCPP_INFO(this->get_logger(),
                        "SEG %u target: %.3f %.3f %.3f %.3f F=%.3f",
                        msg.segment_id,
                        msg.target_positions[0],
                        msg.target_positions[1],
                        msg.target_positions[2],
                        msg.target_positions[3],
                        msg.max_velocity);
        }

        RCLCPP_INFO(this->get_logger(),
                    "Loaded %ld segments",
                    program_segments_.size());
    }

    void startExecution()
    {
        if (start_received_)
        {
            RCLCPP_WARN(this->get_logger(), "START ignored (already started)");
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "[PM] START acknowledged (firmware owns execution)");

        start_received_ = true;
    }

    void stateCallback(const wbp_interfaces::msg::MachineState::SharedPtr msg)
    {
        // debug print
        /* RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[PM] system_state=%u",
            msg->system_state); */

        if (!job_submitted_ &&
            msg->system_state == 2 &&
            !msg->job_loaded &&
            !start_received_ &&
            !program_segments_.empty())
        {
            RCLCPP_INFO(this->get_logger(),
                        "[PM] READY → submitting job (UI contract)");

            submitJobIPC();
            job_submitted_ = true;
        }

        if (!start_received_ && msg->system_state == 3)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[PM] Detected EXECUTING state → starting pipeline");

            startExecution();
        }

        if (start_received_ &&
            msg->system_state == 2 && // READY
            !msg->job_loaded)         // job cleared by firmware
        {
            RCLCPP_INFO(this->get_logger(),
                        "[PM] Job completed → safe reset");

            start_received_ = false;
            job_submitted_ = false;
            program_segments_.clear();
        }
    }

    void submitJobIPC()
    {
        RCLCPP_INFO(this->get_logger(), "[PM DEBUG] submitJobIPC() called");

        if (!job_ipc_)
        {
            RCLCPP_ERROR(this->get_logger(), "Job IPC not available");
            return;
        }

        if (job_ipc_->magic != WBP_JOB_IPC_MAGIC)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid IPC magic");
            return;
        }

        if (job_submitted_)
        {
            return;
        }

        std::vector<wbp_interfaces::Segment> fw_segments;

        for (const auto &seg : program_segments_)
        {
            wbp_interfaces::Segment s;

            s.layer = 0;
            s.index_in_layer = seg.segment_id;

            // assuming order: x y z extrusion
            s.x = seg.target_positions[0];
            s.y = seg.target_positions[1];
            s.z = seg.target_positions[2];
            s.extrusion = seg.target_positions[3];

            // 🔥 TEMP FEEDRATE (until G-code parser)
            s.feedrate = seg.max_velocity; // or fixed value like 0.05

            fw_segments.push_back(s);
        }

        size_t bytes = fw_segments.size() * sizeof(wbp_interfaces::Segment);

        std::memcpy(job_ipc_->job_blob, fw_segments.data(), bytes);
        job_ipc_->size = bytes;

        // notify firmware
        job_ipc_->ingress_complete = 1; // 🔥 immediate (full job ready)
        job_ipc_->sequence++;

        RCLCPP_INFO(this->get_logger(),
                    "[PM] Job written to IPC: segments=%zu size=%zu",
                    fw_segments.size(), bytes);
    }

    void loadGcode(const std::string &file)
    {
        auto lines = parse_gcode(file);

        program_segments_.clear();

        double cx = 0, cy = 0, cz = 0;
        double ce = 0;
        double cf = 0.05; // default feedrate (m/s)

        uint32_t id = 1;

        for (auto &g : lines)
        {
            if (g.has_x)
                cx = g.x;
            if (g.has_y)
                cy = g.y;
            if (g.has_z)
                cz = g.z;

            double extrusion = 0.0;

            if (g.has_e)
            {
                extrusion = g.e - ce;
                ce = g.e;
            }

            if (g.has_f)
            {
                cf = g.f / 1000.0 / 60.0; // mm/min → m/s
            }

            // skip empty lines
            if (!g.has_x && !g.has_y && !g.has_z)
                continue;

            wbp_interfaces::msg::MotionSegment seg;

            seg.segment_id = id++;

            seg.joint_names = {
                "base_to_mast_joint",
                "carriage_to_tool_joint",
                "mast_to_carriage_joint",
                "tool_mount_joint"};

            seg.target_positions = {cx, cy, cz, extrusion};

            seg.max_velocity = cf;
            seg.max_acceleration = 1.0;
            seg.position_tolerance = 0.001;

            // 🔴 NEW — CONTRACT
            seg.v_max = cf;

            // TEMP — will refine later
            if (program_segments_.empty())
                seg.v_start = 0.0;
            else
                seg.v_start = program_segments_.back().v_end;

            // SIMPLE BLENDING RULE
            seg.v_end = cf; // assume straight line

            program_segments_.push_back(seg);

            RCLCPP_INFO(this->get_logger(),
                        "[GCODE] SEG %u: %.3f %.3f %.3f E=%.3f F=%.3f",
                        seg.segment_id, cx, cy, cz, extrusion, cf);
        }

        RCLCPP_INFO(this->get_logger(),
                    "[PM] Loaded %zu segments from G-code",
                    program_segments_.size());

        for (const auto &s : program_segments_)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "SEG %u: TARGET = %.3f",
                         s.segment_id,
                         s.target_positions[0]);
        }

        start_received_ = false;
        job_submitted_ = false;
    }

    std::vector<wbp_interfaces::msg::MotionSegment> program_segments_;

    bool start_received_ = false;

    rclcpp::Subscription<wbp_interfaces::msg::MachineState>::SharedPtr state_sub_;

    std::string yaml_file_;
    bool job_submitted_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionLoader>());
    rclcpp::shutdown();
    return 0;
}

// end of file: ros2_ws/src/wbp_program_manager/src/program_manager.cpp