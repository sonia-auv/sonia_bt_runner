#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common/AiActivationService.h"
#include "ros/ros.h"

class AiActivation : public BT::SyncActionNode
{
public:
    AiActivation(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        const int activation_target_def = 0;
        return {BT::InputPort<int>("activation_target", activation_target_def, "0: OFF, 1: FRONT, 2: BOTTOM, 3: BOTH")};
    }

    BT::NodeStatus tick() override;

private:
    ros::NodeHandle _nh;
    ros::ServiceClient _ai_activation_client;
};