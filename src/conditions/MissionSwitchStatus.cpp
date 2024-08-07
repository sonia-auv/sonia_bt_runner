#include "sonia_bt_runner/conditions/MissionSwitchStatus.hpp"

MissionSwitchStatus::MissionSwitchStatus(const std::string &name)
    : BT::ConditionNode(name, {}), _mission_status(false)
{
    _mission_switch_sub = _nh.subscribe("/provider_kill_mission/mission_switch_msg", 10, &MissionSwitchStatus::update_status, this);
}

BT::NodeStatus MissionSwitchStatus::tick()
{
    ros::spinOnce();
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