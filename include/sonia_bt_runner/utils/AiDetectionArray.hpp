#pragma once

#include <vector>
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"

struct AiDetectionArray {
    std::vector<AiDetection> detection_array;
};

namespace BT
{
    template <> inline AiDetectionArray convertFromString(StringView str)
    {
        auto parts = splitString(str, ';');
        AiDetectionArray output;
        for (size_t i = 0; i < parts.size(); i++)
        {
            output.detection_array.push_back(convertFromString<AiDetection>(parts[i]));
        }
        return output;
    }
} // namespace BT
