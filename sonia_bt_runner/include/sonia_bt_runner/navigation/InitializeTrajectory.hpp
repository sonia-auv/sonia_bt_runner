#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/Trajectory.hpp"

namespace navigation{
    class InitializeTrajectory : public BT::SyncActionNode
    {
    public:
        InitializeTrajectory(const std::string &name, const BT::NodeConfig &config);
        ~InitializeTrajectory() override = default;
        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<Trajectory>("trajectory"),
            };
        }

        BT::NodeStatus tick() override;
    };
}