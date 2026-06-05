// starts here file: firmware/src/job_ingress.cpp
#include "firmware/job_ingress.hpp"

#include <cstring>
#include <vector>
#include <iostream>

#include "firmware/job.hpp"
#include "firmware/job_store.hpp"
#include "firmware/machine_state.hpp"

#include "firmware/job_ingress_state.hpp"

namespace firmware
{

    struct ValidationResult
    {
        bool ok;
        uint32_t schema_version;
        const char *reason;
    };

    // ---- Forward declarations ----
    extern Job g_active_job;
    Job decode_job(const std::vector<uint8_t> &);
    ValidationResult validate_job(const Job &);
    uint64_t allocate_job_id();
    void compute_job_hash(const Job &, char out[64]);

    MachineState &get_machine_state();
    JobStore &get_job_store();

    // ---- Implementation ----

    JobSubmitResult submit_job(const std::vector<uint8_t> &blob,
                               const std::string &)
    {
        auto &ms = get_machine_state();
        ms.last_completion_reason = CompletionReason::NONE;
        JobSubmitResult out{};

        std::cerr << "[DEBUG] sizeof(Segment)=" << sizeof(Segment) << "\n";

        std::cerr << "[FW] submit_job() called\n";
        auto &store = get_job_store();
        auto &js = get_job_ingress_state();

        if (!ms.operator_actions.can_submit_job)
        {
            std::strncpy(js.last_rejection, "operator_recovery_required", 127);
            out.rejection_reason = js.last_rejection;
            return out;
        }

        if (store.has_job())
        {
            std::strncpy(js.last_rejection, "job_already_present", 127);
            out.rejection_reason = js.last_rejection;
            return out;
        }

        if (ms.system != SystemState::READY)
        {
            const char *reason =
                (ms.system == SystemState::RECOVERING)
                    ? "firmware_recovering"
                    : "firmware_not_idle";

            std::strncpy(js.last_rejection, reason, 127);
            out.rejection_reason = js.last_rejection;
            return out;
        }

        // Job job = decode_job(blob);

        Job job{};

        try
        {
            job = decode_job(blob);

            if (job.total_segments == 0)
            {
                std::strncpy(js.last_rejection, "empty_job", 127);
                out.rejection_reason = js.last_rejection;
                return out;
            }

            // 🔍 DEBUG: dump decoded segments
            std::cerr << "[DECODE DEBUG] ---- SEGMENTS ----\n";

            for (size_t i = 0; i < job.total_segments; i++)
            {
                std::cerr << "[DECODE DEBUG] seg=" << i
                          << " x=" << job.segments[i].x
                          << " y=" << job.segments[i].y
                          << " z=" << job.segments[i].z
                          << " E=" << job.segments[i].extrusion
                          << " F=" << job.segments[i].feedrate
                          << std::endl;
            }

            std::cerr << "[DECODE DEBUG] ------------------\n";
        }
        catch (const std::exception &e)
        {
            std::cerr << "[JOB ERROR] decode failed: " << e.what() << "\n";

            std::strncpy(js.last_rejection, "invalid_job_blob", 127);
            out.rejection_reason = js.last_rejection;

            return out;
        }

        // 🔥 CRITICAL FIX
        job.total_segments = job.total_segments;
        job.received_segments = job.total_segments;
        job.ingress_completed = true;
        job.segments_exhausted = true;

        std::cerr << "[JOB DEBUG] total_segments = "
                  << job.total_segments << "\n";

        // 🔥 VALIDATE FEEDRATE FOR ALL SEGMENTS
        for (size_t i = 0; i < job.total_segments; i++)
        {
            if (job.segments[i].feedrate <= 0.0)
            {
                std::cerr << "[JOB ERROR] segment " << i
                          << " has invalid feedrate\n";
            }
        }

        ValidationResult v = validate_job(job);
        if (!v.ok)
        {
            std::strncpy(js.last_rejection, v.reason, 127);
            out.rejection_reason = js.last_rejection;
            return out;
        }

        uint64_t jid = allocate_job_id();
        char hash[64]{};
        compute_job_hash(job, hash);

        // --- PERSISTENCE ---
        store.set_job(job, jid, hash, v.schema_version);

        // --- EXECUTION BINDING ---
        g_active_job = job;

        g_active_job.received_segments = g_active_job.total_segments;
        g_active_job.ingress_completed = true;
        g_active_job.segments_exhausted = true;

        std::cerr << "[JOB DEBUG] AFTER COPY received_segments="
                  << g_active_job.received_segments << "\n";

        std::cerr << "[JOB DEBUG] g_active_job.received_segments="
                  << g_active_job.received_segments << "\n";

        // --- MACHINE STATE (CRITICAL) ---
        ms.job_present = true;
        ms.job_id = jid;
        std::cerr << "[FW STATE] job_present=" << ms.job_present
                  << " job_id=" << ms.job_id << std::endl;

        ms.job_schema_version = v.schema_version;
        std::memcpy(ms.job_hash, hash, 64);

        // Reset execution-visible progress
        ms.job.layer = 0;
        ms.job.segment = 0;
        ms.job.state = JobState::NONE;

        // --- JOB INGRESS STATE (IPC SOURCE OF TRUTH) ---
        js.job_loaded = true;
        js.job_id = jid;
        js.schema_version = v.schema_version;
        std::memcpy(js.job_hash, hash, 64);
        js.last_rejection[0] = '\0';

        out.accepted = true;
        out.job_id = jid;
        out.job_hash = hash;

        return out;
    }

} // namespace firmware

// end of firmware/src/job_ingress.cpp