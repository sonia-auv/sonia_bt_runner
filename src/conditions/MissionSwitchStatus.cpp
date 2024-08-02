#include "sonia_bt_runner/conditions/MissionSwitchStatus.hpp"

BT::NodeStatus MissionSwitchStatus::tick()
{
    if (_mission_status)
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void MissionSwitchStatus::update_status(const std_msgs::Bool::ConstPtr &msg)
{
    _mission_status = msg->data;
}