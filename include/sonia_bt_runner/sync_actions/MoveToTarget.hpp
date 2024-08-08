#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"


class MoveToTarget : public BT::SyncActionNode
{
public:
    MoveToTarget(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {

        return {
            BT::InputPort<AiDetection>("aiObj"),
            BT::InputPort<float>("offset"),
            BT::BidirectionalPort<Trajectory>("trajectory"),
        };
    }

    BT::NodeStatus tick() override;
};