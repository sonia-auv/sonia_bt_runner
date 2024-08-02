#include "sonia_bt_runner/sync_actions/TareDepth.hpp"

TareDepth::TareDepth(const std::string &name)
    : BT::SyncActionNode(name, {})
{
    _depth_client = _nh.serviceClient<std_srvs::Empty>("/provider_depth/tare");
}

BT::NodeStatus TareDepth::tick()
{
    std_srvs::Empty tare;

    if (_depth_client.call(tare))
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}