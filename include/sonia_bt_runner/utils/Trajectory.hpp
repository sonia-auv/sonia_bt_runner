#pragma once

#include <vector>
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/TrajectoryPose.hpp"

struct Trajectory
{
    std::vector<TrajectoryPose> trajectory;
};

namespace BT
{
    template <> inline Trajectory convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        Trajectory output;
        for (size_t i = 0; i < parts.size(); i++)
        {
            output.trajectory.push_back(convertFromString<TrajectoryPose>(parts[i]));
        }
        return output;
    }

} // end namespace BT