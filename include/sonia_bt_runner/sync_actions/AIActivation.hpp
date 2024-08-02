#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common/AiActivationService.h"
#include "ros/ros.h"

class AIActivation : public BT::SyncActionNode
{
public:
    AIActivation(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)
    {
        _ai_activation_client = _nh.serviceClient<sonia_common::AiActivationService>("/proc_vision/ai_activation");
    }

    static BT::PortsList providedPorts()
    {
        int activation_target_def = 0;
        return {BT::InputPort<int>("activation_target", activation_target_def, "0: OFF, 1: FRONT, 2: BOTTOM, 3: BOTH")};
    }
    BT::NodeStatus tick() override
    {
        int action = 0;
        getInput("activation_target", action);

        sonia_common::AiActivationService srv;

        srv.request.ai_activation = action;

        if (_ai_activation_client.call(srv))
        {
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::FAILURE;
    }

private:
    ros::NodeHandle _nh;
    ros::ServiceClient _ai_activation_client;
};