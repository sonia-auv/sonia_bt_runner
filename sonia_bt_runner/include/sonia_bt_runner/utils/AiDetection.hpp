#pragma once
#include <string>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/json_export.h"

struct AiDetection
{
    std::string classification;
    float confidence;
    float distance;
    float angle_teta;
    float angle_alpha;
    float distance_teta;
    float distance_beta;
    float top_left_x;
    float top_left_y;
    float top_right_x;
    float top_right_y;
    float bottom_left_x;
    float bottom_left_y;
    float bottom_right_x;
    float bottom_right_y;
};

// Lets Groot2 and opengroot show the value while debugging
BT_JSON_CONVERTER(AiDetection, detection)
{
    add_field("classification", &detection.classification);
    add_field("confidence", &detection.confidence);
    add_field("distance", &detection.distance);
    add_field("angle_teta", &detection.angle_teta);
    add_field("angle_alpha", &detection.angle_alpha);
    add_field("distance_teta", &detection.distance_teta);
    add_field("distance_beta", &detection.distance_beta);
    add_field("top_left_x", &detection.top_left_x);
    add_field("top_left_y", &detection.top_left_y);
    add_field("top_right_x", &detection.top_right_x);
    add_field("top_right_y", &detection.top_right_y);
    add_field("bottom_left_x", &detection.bottom_left_x);
    add_field("bottom_left_y", &detection.bottom_left_y);
    add_field("bottom_right_x", &detection.bottom_right_x);
    add_field("bottom_right_y", &detection.bottom_right_y);
}

namespace BT
{
    template <> inline AiDetection convertFromString(StringView str)
    {
        auto parts = splitString(str, ',');
        if (parts.size() != 11)
        {
            throw RuntimeError("Invalid Input");
        }
        // The angles are not in the text: they must be zero, not whatever was in memory
        AiDetection output{};
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