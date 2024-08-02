#include "sonia_bt_runner/sync_actions/InitTraj.hpp"

InitTraj::InitTraj(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus InitTraj::tick()
{
    setOutput("trajectory", "");
    return BT::NodeStatus::SUCCESS;
}