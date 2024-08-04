#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "ros/ros.h"

#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"

class GetAlignStrafe : public BT::SyncActionNode
{
public:
    GetAlignStrafe(const std::string name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<AiDetection>("detection_object"),
            BT::InputPort<int>("target_x"),
            BT::InputPort<int>("target_y"),
            BT::BidirectionalPort<Trajectory>("traj")};
    }

    BT::NodeStatus tick() override;

private:
    const float _pixel_to_meters = 0.0024;
};
