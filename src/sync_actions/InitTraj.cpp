#include "sonia_bt_runner/sync_actions/InitTraj.hpp"

BT::NodeStatus InitTraj::tick()
{
    setOutput("trajectory", "");
    return BT::NodeStatus::SUCCESS;
}