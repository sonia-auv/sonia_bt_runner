#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/Trajectory.hpp"

class InitTraj : public BT::SyncActionNode
{
public:
    InitTraj(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<Trajectory>("trajectory"),
        };
    }

    BT::NodeStatus tick() override;
};