#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/Trajectory.hpp"

#define INITIALIZE_TRAJECTORY_TRAJECTORY_PARAM_NAME "trajectory"
#define INITIALIZE_TRAJECTORY_TRAJECTORY_PARAM_TYPE Trajectory

namespace navigation{
    class InitializeTrajectory : public BT::SyncActionNode
    {
    public:
        InitializeTrajectory(const std::string &name, const BT::NodeConfig &config);
        ~InitializeTrajectory() override = default;
        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<INITIALIZE_TRAJECTORY_TRAJECTORY_PARAM_TYPE>(INITIALIZE_TRAJECTORY_TRAJECTORY_PARAM_NAME),
            };
        }

        BT::NodeStatus tick() override;
    };
}