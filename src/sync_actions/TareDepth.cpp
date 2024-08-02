#include "sonia_bt_runner/sync_actions/TareDepth.hpp"

BT::NodeStatus TareDepth::tick()
{
    std_srvs::Empty tare;

    if (_depth_client.call(tare))
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}