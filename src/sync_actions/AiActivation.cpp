#include "sonia_bt_runner/sync_actions/AiActivation.hpp"

AiActivation::AiActivation(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
{
    _ai_activation_client = _nh.serviceClient<sonia_common::AiActivationService>("/proc_vision/ai_activation");
}

BT::NodeStatus AiActivation::tick()
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