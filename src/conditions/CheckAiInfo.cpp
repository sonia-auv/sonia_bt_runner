#include "sonia_bt_runner/conditions/CheckAiInfo.hpp"

CheckAiInfo::CheckAiInfo(const std::string name, const BT::NodeConfig &config)
    : BT::ConditionNode(name, config)
{
    _ai_info_front_sub = _nh.subscribe("/proc_vision/front/classification", 10, &CheckAiInfo::ai_info_front_callback, this);
    _ai_info_bottom_sub = _nh.subscribe("/proc_vision/bottom/classification", 10, &CheckAiInfo::ai_info_bottom_callback, this);
}

BT::NodeStatus CheckAiInfo::tick()
{
    int camera = 0;
    getInput<int>("camera", camera);
    std::string classification = "";
    getInput<std::string>("classification", classification);
    float confidence = 0.8;
    getInput<float>("confidence", confidence);
    int min_count = 1;
    getInput<int>("min_count", min_count);
    int max_count = 1;
    getInput<int>("max_count", max_count);

    sonia_common::DetectionArray value;
    ros::Duration(1).sleep();
    ros::spinOnce();

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
        ROS_INFO("BAD_CAM");
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
        ROS_INFO("No class found");
        return BT::NodeStatus::FAILURE;
    }

    // Check confidence
    std::vector<sonia_common::Detection> confident_classifications;
    for (size_t i = 0; i < found_classifications.size(); i++)
    {
        auto tmp = found_classifications[i];
        if (tmp.confidence >= confidence)
        {
            ROS_INFO("%s", tmp.class_name.c_str());
            confident_classifications.push_back(tmp);
        }
    }

    // Check if count is what we want
    if (confident_classifications.size() < min_count || confident_classifications.size() > max_count)
    {
        ROS_INFO("Class count bad!");
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
        obj.left = tmp.left;
        obj.bottom = tmp.bottom;
        obj.right = tmp.right;
        output.detection_array.push_back(obj);
    }
    setOutput("ai_info", output);
    return BT::NodeStatus::SUCCESS;
}

void CheckAiInfo::ai_info_front_callback(const sonia_common::DetectionArray &msg)
{
    _last_front = msg;
}

void CheckAiInfo::ai_info_bottom_callback(const sonia_common::DetectionArray &msg)
{
    _last_bottom = msg;
}
