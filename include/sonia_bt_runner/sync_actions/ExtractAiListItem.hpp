#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"

class ExtractAiListItem : public BT::SyncActionNode
{
public:
    ExtractAiListItem(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<AiDetectionArray>("array"),
            BT::InputPort<int>("Index"),
            BT::OutputPort<AiDetection>("aiObject")};
    }

    BT::NodeStatus tick() override;
};