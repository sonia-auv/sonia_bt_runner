#include "sonia_bt_runner/sync_actions/GetAiInfo.hpp"

BT::NodeStatus GetAiInfo::tick()
{
    uint8_t camera = 0;
    getInput<uint8_t>("camera", camera);
    std::string classification = "";
    getInput<std::string>("classification", classification);
    float confidence= 0.8;
    getInput<float>("confidence", confidence);
    uint8_t min_count = 1;
    getInput<uint8_t>("min_count", min_count);
    uint8_t max_count = 1;
    getInput<uint8_t>("max_count", max_count);

    sonia_common::DetectionArray value;

    if (camera == 0) // FRONT
    {
        value = _last_front;
    }
    else if (camera == 1) // BOTTOM
    {
        value = _last_bottom;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }

    // check if wanted classification is present
    std::vector<sonia_common::Detection> found_classifications;
    for (size_t i = 0; i < value.detected_object.size(); i++)
    {
        auto tmp = value.detected_object[i];
        if (tmp.class_name == classification)
        {
            found_classifications.push_back(tmp);
        }
    }

    if (found_classifications.size() == 0)
    {
        return BT::NodeStatus::FAILURE;
    }

    // Check confidence
    std::vector<sonia_common::Detection> confident_classifications;
    for (size_t i = 0; i < found_classifications.size(); i++)
    {
        auto tmp = found_classifications[i];
        if (tmp.confidence >= confidence)
        {
            confident_classifications.push_back(tmp);
        }
    }
    
    // Check if count is what we want
    if (confident_classifications.size() < min_count || confident_classifications.size() > max_count)
    {
        return BT::NodeStatus::FAILURE;
    }

    AiDetectionArray output;
    for (size_t i = 0; i < confident_classifications.size(); i++)
    {
        auto tmp = confident_classifications[i];
        AiDetection obj;
        obj.classification = tmp.class_name;
        obj.distance = tmp.distance;
        obj.top = tmp.top;
        obj.left= tmp.left;
        obj.bottom = tmp.bottom;
        obj.right = tmp.right;
        output.detection_array.push_back(obj);
    }
    setOutput("ai_info", output);
    return BT::NodeStatus::SUCCESS;
}

void GetAiInfo::ai_info_front_callback(const sonia_common::DetectionArray &msg)
{
    _last_front = msg;
}

void GetAiInfo::ai_info_bottom_callback(const sonia_common::DetectionArray &msg)
{
    _last_bottom = msg;
}
