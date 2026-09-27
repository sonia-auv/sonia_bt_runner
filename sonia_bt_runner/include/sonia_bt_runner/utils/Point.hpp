#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/json_export.h"

struct Point
{
    double x;
    double y;
    double z;
};

// Lets Groot2 and opengroot show the value while debugging
BT_JSON_CONVERTER(Point, point)
{
    add_field("x", &point.x);
    add_field("y", &point.y);
    add_field("z", &point.z);
}

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