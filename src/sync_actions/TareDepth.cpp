#include "sonia_bt_runner/sync_actions/TareDepth.hpp"
TareDepth::TareDepth(const std::string &name)
    : BT::SyncActionNode(name, {}),
    auv {std::getenv("AUV")}
{
    _depth_client = _nh.serviceClient<std_srvs::Empty>("/provider_depth/tare");
}

BT::NodeStatus TareDepth::tick()
{
    std_srvs::Empty tare;

    try
    {
        if (strcmp(auv, "AUV7") ||_depth_client.call(tare))
        {
            return BT::NodeStatus::SUCCESS;
        }
    }
    catch(const std::exception& e)
    {
        ROS_WARN("Segmentation fault. set environnement variable");
    }
    
    
    return BT::NodeStatus::FAILURE;
}