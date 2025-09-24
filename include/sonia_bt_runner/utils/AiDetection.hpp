#pragma once
#include <string>
#include "behaviortree_cpp/behavior_tree.h"

struct AiDetection
{
    std::string classification;
    float confidence;
    float distance;
    float angle_teta;
    float angle_alpha;
    float distance_teta;
    float top_right_y;
    float bottom_left_x;
    float bottom_left_y;
    float bottom_right_x;
    float bottom_right_y;
};

namespace BT
{
    template <> inline AiDetection convertFromString(StringView str)
    {
        auto parts = splitString(str, ',');
        if (parts.size() != 11)
        {
            throw RuntimeError("Invalid Input");
        }
        AiDetection output;
        output.classification = parts[0];
        output.confidence = convertFromString<float>(parts[1]);
        output.distance = convertFromString<float>(parts[2]);
        output.top_left_x= convertFromString<float>(parts[3]);
        output.top_left_y= convertFromString<float>(parts[4]);
        output.top_right_x= convertFromString<float>(parts[5]);
        output.top_right_y= convertFromString<float>(parts[6]);
        output.bottom_left_x= convertFromString<float>(parts[7]);
        output.bottom_left_y= convertFromString<float>(parts[8]);
        output.bottom_right_x= convertFromString<float>(parts[9]);
        output.bottom_right_y= convertFromString<float>(parts[10]);

        return output;
    }
} // namespace BT