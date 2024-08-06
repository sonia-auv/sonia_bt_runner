#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "ros/ros.h"

class GetAngle : public BT::SyncActionNode
{
public:
    GetAngle(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        const float error_marg_def = 0.1;
        return {
            BT::InputPort<AiDetection>("aiObj"),
            BT::InputPort<float>("ideal_ratio"),
            BT::InputPort<float>("error_margin", error_marg_def, ""),
            BT::OutputPort<float>("detected_angle")};
    }

    BT::NodeStatus tick() override;
};