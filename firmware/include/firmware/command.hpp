// start of firmware/include/firmware/command.hpp
#pragma once
#include <cstdint>

namespace firmware
{

    enum class CommandType : uint8_t
    {
        // START_JOB,
        JOB_START,
        SET_LAYER,
        PRINT_SEGMENT,
        JOB_END
    };

    struct Command
    {
        CommandType type;
        union
        {
            struct
            {
                uint32_t job_id;
            } job_start;
            struct
            {
                uint32_t layer;
            } set_layer;
            struct
            {
                uint32_t seg, x, y;
                double z;
            } print;
        };

        static Command JobStart(uint32_t id)
        {
            Command c{};
            c.type = CommandType::JOB_START;
            c.job_start.job_id = id;
            return c;
        }

        static Command SetLayer(uint32_t l)
        {
            Command c{};
            c.type = CommandType::SET_LAYER;
            c.set_layer.layer = l;
            return c;
        }

        static Command Print(uint32_t s, uint32_t x, uint32_t y, double z)
        {
            Command c{};
            c.type = CommandType::PRINT_SEGMENT;
            c.print = {s, x, y, z};
            return c;
        }

        static Command End()
        {
            Command c{};
            c.type = CommandType::JOB_END;
            return c;
        }
    };

}

// end of firmware/include/firmware/command.hpp