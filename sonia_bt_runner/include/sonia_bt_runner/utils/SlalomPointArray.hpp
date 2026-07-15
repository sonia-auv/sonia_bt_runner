#pragma once

#include <string>
#include <vector>
#include "behaviortree_cpp/behavior_tree.h"

struct SlalomPoint
{
    float x;
    float y;
    std::string classification;
    float confidence;
};

struct SlalomPointArray
{
    std::vector<SlalomPoint> points;
};

namespace BT
{
    template <> inline SlalomPointArray convertFromString(StringView str)
    {
        // We expect "x,y" pairs separated by semicolons, an empty string gives an empty array
        SlalomPointArray output;
        if (str.empty())
        {
            return output;
        }
        auto parts = splitString(str, ';');
        for (size_t i = 0; i < parts.size(); i++)
        {
            auto values = splitString(parts[i], ',');
            if (values.size() != 2)
            {
                throw RuntimeError("Invalid Input");
            }
            SlalomPoint point{};
            point.x = convertFromString<float>(values[0]);
            point.y = convertFromString<float>(values[1]);
            output.points.push_back(point);
        }
        return output;
    }
} // namespace BT
