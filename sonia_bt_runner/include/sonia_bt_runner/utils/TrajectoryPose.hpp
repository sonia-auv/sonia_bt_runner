#pragma once

#include <vector>
#include "behaviortree_cpp/behavior_tree.h"

struct TrajectoryPose
{
    float positionX;
    float positionY;
    float positionZ;
    float orientationX;
    float orientationY;
    float orientationZ;
    int frame;
    int speed;
    int precision;
    bool long_rotation;
};

namespace BT
{
    template <> inline TrajectoryPose convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ',');
        if (parts.size() != 10)
        {
            throw RuntimeError("invalid input)");
        }
        TrajectoryPose output;
        output.positionX = convertFromString<float>(parts[0]);
        output.positionY = convertFromString<float>(parts[1]);
        output.positionZ = convertFromString<float>(parts[2]);
        output.orientationX = convertFromString<float>(parts[3]);
        output.orientationY = convertFromString<float>(parts[4]);
        output.orientationZ = convertFromString<float>(parts[5]);
        output.frame = convertFromString<int>(parts[6]);
        output.speed = convertFromString<int>(parts[7]);
        output.precision = convertFromString<int>(parts[8]);
        output.long_rotation = convertFromString<bool>(parts[9]);
        
        return output;
    }
} // end namespace BT