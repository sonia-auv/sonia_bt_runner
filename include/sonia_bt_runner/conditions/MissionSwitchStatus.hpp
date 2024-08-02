#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"

static bool mission_status = false;

class MissionSwitchStatus : public BT::ConditionNode
{
public:
    MissionSwitchStatus(const std::string &name)
        : BT::ConditionNode(name, {}), _mission_status(false)
    {
        _mission_switch_sub = _nh.subscribe("/provider_kill_mission/mission_switch_msg", 10, &MissionSwitchStatus::update_status, this);
    }

    BT::NodeStatus tick() override;

private:
    void update_status(const std_msgs::Bool::ConstPtr &msg);

    ros::NodeHandle _nh;
    ros::Subscriber _mission_switch_sub;
    bool _mission_status;
};