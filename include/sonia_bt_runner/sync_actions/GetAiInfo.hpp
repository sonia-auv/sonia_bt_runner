#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "ros/ros.h"
#include "sonia_common/Detection.h"
#include "sonia_common/DetectionArray.h"

#include "sonia_bt_runner/utils/AiDetectionArray.hpp"

class GetAiInfo : public BT::SyncActionNode
{
public:
    GetAiInfo(const std::string name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)
    {
        _ai_info_front_sub = _nh.subscribe("/proc_vision/front/classification", 10, &GetAiInfo::ai_info_front_callback, this);
        _ai_info_bottom_sub = _nh.subscribe("/proc_vision/bottom/classification", 10, &GetAiInfo::ai_info_bottom_callback, this);
    }

    static BT::PortsList providedPorts()
    {
        int camera_def = 0;
        int count_def = 1;
        float confidence_def = 0.8;
        return {
            BT::InputPort<std::string>("classification", ""),
            BT::InputPort<int>("camera", camera_def, "0: Front, 1: Bottom"),
            BT::InputPort<int>("min_count", count_def, ""),
            BT::InputPort<int>("max_count", count_def, ""),
            BT::InputPort<float>("confidence", confidence_def, ""),
            BT::OutputPort<AiDetectionArray>("ai_info")};
    }

    BT::NodeStatus tick() override;

private:
    void ai_info_front_callback(const sonia_common::DetectionArray &msg);
    void ai_info_bottom_callback(const sonia_common::DetectionArray &msg);

    ros::NodeHandle _nh;
    ros::Subscriber _ai_info_front_sub;
    ros::Subscriber _ai_info_bottom_sub;
    sonia_common::DetectionArray _last_front;
    sonia_common::DetectionArray _last_bottom;
};