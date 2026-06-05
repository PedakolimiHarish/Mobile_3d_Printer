// start of: firmware/src/execution_loop.cpp

#include "firmware/execution_loop.hpp"
#include "firmware/motion/motion_controller.hpp"
#include "firmware/extruder/extruder_controller.hpp"
#include "firmware/extruder/extruder_types.hpp"
#include "firmware/job.hpp"
#include <cassert>
#include <iostream>

#include <chrono>
#include <thread>
namespace firmware
{

    ExecutionResult execution_step(
        ExecutionContext &ctx,
        const Job &job,
        MotionController &motion,
        ExtruderController &extruder)
    {

        std::cerr << "[EXEC] phase=" << int(ctx.phase)
                  << " owned=" << ctx.owned_by_executor
                  << std::endl;

        if (!ctx.owned_by_executor)
        {
            return ExecutionResult::BLOCKED;
        }

        if (ctx.abort_in_progress)
        {
            return ExecutionResult::BLOCKED;
        }

        // 🔒 STOP LOOP AFTER COMPLETION
        if (ctx.phase != ExecutionPhase::RUNNING)
        {
            return ExecutionResult::BLOCKED;
        }

        assert(ctx.owned_by_executor && "ExecutionContext mutated outside executor");
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        static uint32_t last_cursor = 0;

        if (ctx.cursor == 0 && ctx.phase == ExecutionPhase::RUNNING)
        {
            last_cursor = 0; // 🔒 new execution epoch
        }

        assert(ctx.cursor >= last_cursor);
        last_cursor = ctx.cursor;

        /* -------- Execution gate -------- */
        if (ctx.phase == ExecutionPhase::PAUSED)
        {
            return ExecutionResult::BLOCKED;
        }

        /* if (ctx.phase != ExecutionPhase::RUNNING)
        {
            return ExecutionResult::BLOCKED;
        } */

        // ===== HARD COMPLETION GUARD =====
        if (ctx.cursor >= job.total_segments)
        {
            if (motion.is_ready())
            {
                /* ctx.owned_by_executor = false; // 🔒 FIX
                ctx.phase = ExecutionPhase::IDLE; */
                return ExecutionResult::COMPLETED;
            }
            return ExecutionResult::BLOCKED;
        }

        /* ===== COMPLETION CHECK (Phase 13.A – stub) ===== */
        /* ===== COMPLETION CHECK (Phase 13.A – slow stub) ===== */

        /* -------- Extruder settle -------- */
        if (!extruder.is_ready())
        {
            ExtruderResult er = extruder.poll();

            switch (er)
            {
            case ExtruderResult::IN_PROGRESS:
            case ExtruderResult::ACCEPTED:
            case ExtruderResult::REJECTED:
                return ExecutionResult::BLOCKED;

            case ExtruderResult::COMPLETED:
                break;

            case ExtruderResult::FAULTED:
                return ExecutionResult::FAULTED;
            }
        }

        /* -------- Motion settle -------- */
        if (!motion.is_ready())
        {
            MotionResult mr = motion.poll();

            switch (mr)
            {
            case MotionResult::IN_PROGRESS:

            case MotionResult::ACCEPTED:
                return ExecutionResult::PROGRESSED;

            case MotionResult::COMPLETED:

                break;

            case MotionResult::REJECTED:
                return ExecutionResult::BLOCKED;

            case MotionResult::FAULTED:
                return ExecutionResult::FAULTED;
            }
        }

        // =========================================================
        // 🔒 STREAMING SEGMENT FEED (NEW CORE LOGIC)
        // =========================================================

        /* while (motion.is_ready() &&
               ctx.cursor < job.total_segments)
        {
            const auto &seg = job.segments[ctx.cursor];

            MotionCommand mcmd{};
            mcmd.intent = MotionIntent::TRAVEL;

            // 🔥 CRITICAL FIX — COPY GEOMETRY
            mcmd.x = seg.x;
            mcmd.y = seg.y;
            mcmd.z = seg.z;

            double feedrate = seg.feedrate;
            mcmd.extrusion = seg.extrusion;

            if (feedrate <= 0.0)
            {
                std::cerr << "[EXEC ERROR] Invalid feedrate\n";
                return ExecutionResult::FAULTED;
            }

            // 🔥 already correct
            mcmd.feedrate = seg.feedrate;

            std::cerr << "[EXEC] submit seg=" << ctx.cursor
                      << " F=" << feedrate << "\n";

            if (motion.submit(mcmd) == MotionResult::FAULTED)
            {
                return ExecutionResult::FAULTED;
            }

            ctx.cursor++; // 🔥 CRITICAL: increment on SUBMIT, not completion
        } */

        if (motion.is_ready() &&
            ctx.cursor < job.total_segments)
        {
            const auto &seg = job.segments[ctx.cursor];

            MotionCommand mcmd{};
            mcmd.intent = MotionIntent::TRAVEL;

            mcmd.x = seg.x;
            mcmd.y = seg.y;
            mcmd.z = seg.z;

            mcmd.feedrate = seg.feedrate;
            mcmd.extrusion = seg.extrusion;

            if (motion.submit(mcmd) == MotionResult::FAULTED)
            {
                return ExecutionResult::FAULTED;
            }

            ctx.cursor++;
        }

        /* Debug output */
        static uint32_t last_dbg_cursor = UINT32_MAX;

        if (ctx.cursor != last_dbg_cursor)
        {
            last_dbg_cursor = ctx.cursor;

            std::cerr << "[EXEC DBG] cursor=" << ctx.cursor << "\n";
        }

        std::cerr << "[EXEC CHECK] cursor=" << ctx.cursor
                  << " total=" << job.total_segments
                  << " received=" << job.received_segments
                  << " ingress=" << job.ingress_completed
                  << "\n";
        /* Debug output */

        return ExecutionResult::PROGRESSED;
    }

} // namespace firmware

// end of file: firmware/src/execution_loop.cpp