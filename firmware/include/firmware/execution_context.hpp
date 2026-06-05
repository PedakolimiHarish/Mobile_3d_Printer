// firmware/include/firmware/execution_context.hpp

// EXECUTION INVARIANTS (Phase 22.C)
//
// I1. cursor == 0  <=> execution has not started
// I2. phase == IDLE implies no motion may be emitted
// I3. completion_emitted == true implies phase == COMPLETED
// I4. START always resets execution context
// I5. PAUSE preserves cursor and phase
// I6. RESUME never resets cursor
// I7. CLEAR_JOB guarantees execution context is idle

#pragma once
#include <cstdint>
#include <cassert>
#include "firmware/pause_snapshot.hpp"
namespace firmware
{
    /**
     * ExecutionPhase represents the internal execution status
     * of a job. This is NOT a lifecycle or authority state.
     */
    enum class ExecutionPhase : uint8_t
    {
        IDLE,       // No active execution
        RUNNING,    // Actively executing
        PAUSED,     // Execution halted, resumable
        COMPLETING, // Finalization in progress
        COMPLETED   // Execution finished successfully
    };

    /**
     * ExecutionContext is pure execution state.
     * It encodes everything required to deterministically
     * continue or resume execution.
     */
    /*     struct ExecutionContext
        {

            bool owned_by_executor{false};
            // Current execution phase (internal only)
            ExecutionPhase phase{ExecutionPhase::IDLE};

            // Cursor into the execution sequence (instruction / segment / step)
            uint32_t cursor{0};

            // Normalized progress within the current execution unit [0.0, 1.0]
            double unit_progress{0.0};

            // bool resumable{false};

            // 🔒 Phase 23.A — execution ownership
            bool armed = false;

            // Phase 13.C
            bool completion_emitted{false};

            // --- PUBLIC, SAFE ---
            void reset_soft(); // used by invariants, faults, aborts

            // --- RESTRICTED ---
            void reset_hard(); // ONLY orchestrator / executor

            void reset()
            {
                cursor = 0;
                unit_progress = 0.0;
                phase = ExecutionPhase::IDLE;
                armed = false;
                // owner = ExecutionOwner::NONE;
                completion_emitted = false;
                owned_by_executor = false;
            }
        }; */

    struct ExecutionContext
    {
        // 🔒 Ownership
        bool owned_by_executor{false};

        // 🔒 Execution state
        ExecutionPhase phase{ExecutionPhase::IDLE};
        uint32_t cursor{0};
        double unit_progress{0.0};

        bool abort_in_progress{false};

        // 🔒 Phase 23.A
        bool armed{false};

        // 🔒 Phase 13.C
        bool completion_emitted{false};

        bool abort_sent = false;

        uint32_t blocked_ticks = 0;

        // ------------------------
        // Reset tiers (Phase 32.B)
        // ------------------------

        // Soft reset: preserve resumability semantics
        // --- PUBLIC, SAFE ---
        inline void reset_soft()
        {
            // Soft reset preserves cursor semantics only if paused/resuming
            phase = ExecutionPhase::IDLE;
            armed = false;
            owned_by_executor = false;
            completion_emitted = false;
            abort_sent = false;
        }

        // Hard reset: lifecycle authority ONLY
        // --- RESTRICTED ---
        inline void reset_hard()
        {
            cursor = 0;
            unit_progress = 0.0;
            phase = ExecutionPhase::IDLE;
            armed = false;
            owned_by_executor = false;
            completion_emitted = false;
            abort_sent = false;
            blocked_ticks = 0;
        }

        inline void reset()
        {
            reset_hard();
        }
    };

    inline void assert_execution_ownership_valid(const ExecutionContext &ctx)
    {
        // Both true is illegal
        assert(!(ctx.armed && ctx.owned_by_executor) &&
               "ExecutionContext cannot be armed and owned simultaneously");
    }

} // namespace firmware
