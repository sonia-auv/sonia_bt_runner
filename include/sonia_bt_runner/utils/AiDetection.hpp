#pragma once
#include <string>
#include "behaviortree_cpp/behavior_tree.h"

struct AiDetection
{
    std::string classification;
    float distance;
    float top;
    float left;
    float bottom;
    float right;
};

namespace BT
{
    template <> inline AiDetection convertFromString(StringView str)
    {
        auto parts = splitString(str, ',');
        if (parts.size() != 6)
        {
            throw RuntimeError("Invalid Input");
        }
        AiDetection output;
        output.classification = parts[0];
        output.distance = convertFromString<float>(parts[1]);
        output.top = convertFromString<float>(parts[2]);
        output.left = convertFromString<float>(parts[3]);
        output.bottom = convertFromString<float>(parts[4]);
        output.right = convertFromString<float>(parts[5]);

        return output;
    }
} // namespace BT
