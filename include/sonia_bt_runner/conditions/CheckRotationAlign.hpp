#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "ros/ros.h"

class CheckRotationAlign : public BT::ConditionNode
{
public:
    CheckRotationAlign(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<AiDetection>("aiObj"),
            BT::InputPort<float>("minRatio"),
            BT::InputPort<float>("maxRatio"),
            BT::OutputPort<int>("status", "0: angle too small, 1: angle ok")};
    }

    BT::NodeStatus tick() override;
};