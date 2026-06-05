// start of file: ros2_ws/src/wbp_program_manager/src/gcode_parser.cpp
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>

struct GCodeLine
{
    double x = 0, y = 0, z = 0;
    double e = 0;
    double f = 0;
    bool has_x = false, has_y = false, has_z = false;
    bool has_e = false, has_f = false;
};

std::vector<GCodeLine> parse_gcode(const std::string &file)
{
    std::ifstream in(file);
    std::vector<GCodeLine> lines;

    std::string line;

    double last_x = 0, last_y = 0, last_z = 0;

    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == ';')
            continue;

        std::stringstream ss(line);
        std::string token;

        GCodeLine g;

        while (ss >> token)
        {
            if (token[0] == 'X')
            {
                g.x = std::stod(token.substr(1));
                g.has_x = true;
            }
            else if (token[0] == 'Y')
            {
                g.y = std::stod(token.substr(1));
                g.has_y = true;
            }
            else if (token[0] == 'Z')
            {
                g.z = std::stod(token.substr(1));
                g.has_z = true;
            }
            else if (token[0] == 'E')
            {
                g.e = std::stod(token.substr(1));
                g.has_e = true;
            }
            else if (token[0] == 'F')
            {
                g.f = std::stod(token.substr(1));
                g.has_f = true;
            }
        }

        if (g.has_x || g.has_y || g.has_z)
        {
            // fill missing values with last known
            if (!g.has_x)
                g.x = last_x;
            if (!g.has_y)
                g.y = last_y;
            if (!g.has_z)
                g.z = last_z;

            // ❗ ONLY push if movement exists
            if (g.x != last_x || g.y != last_y || g.z != last_z)
            {
                lines.push_back(g);

                last_x = g.x;
                last_y = g.y;
                last_z = g.z;
            }
        }
    }

    return lines;
}

// end of file: ros2_ws/src/wbp_program_manager/src/gcode_parser.cpp