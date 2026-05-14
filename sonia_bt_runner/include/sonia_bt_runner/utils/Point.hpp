#pragma once

#include "behaviortree_cpp/behavior_tree.h"

struct Point
{
    double x;
    double y;
    double z;
};

namespace BT
{
    template <> inline Point convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        Point output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
        output.z = convertFromString<double>(parts[2]);
        return output;
    }

} // end namespace BT