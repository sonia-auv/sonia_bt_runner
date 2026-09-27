#pragma once

#include <vector>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/json_export.h"
#include "sonia_bt_runner/utils/TrajectoryPose.hpp"

struct Trajectory
{
    std::vector<TrajectoryPose> trajectory;
};

// Lets Groot2 and opengroot show the value while debugging, each pose written by its own converter
BT_JSON_CONVERTER(Trajectory, trajectory)
{
    add_field("trajectory", &trajectory.trajectory);
}

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