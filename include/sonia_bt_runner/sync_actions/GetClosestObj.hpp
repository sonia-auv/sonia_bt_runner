#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"

class GetClosestObj : public BT::SyncActionNode
{
public:
    GetClosestObj(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<AiDetectionArray>("array"),
            BT::OutputPort<AiDetection>("aiObj")};
    }

    BT::NodeStatus tick() override;
};
