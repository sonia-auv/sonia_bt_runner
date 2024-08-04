#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "ros/ros.h"

class CheckWidthAngle : public BT::ConditionNode
{
public:
    CheckWidthAngle(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<AiDetection>("aiObj"),
            BT::InputPort<float>("minRatio"),
            BT::InputPort<float>("maxRatio")};
    }

    BT::NodeStatus tick() override;
};