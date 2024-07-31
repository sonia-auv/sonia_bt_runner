#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"

static bool mission_status = false;

class MissionSwitchStatus : public BT::ConditionNode
{
public:
    MissionSwitchStatus(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config), _mission_status(false)
    {
        _mission_switch_sub = _nh.subscribe("/provider_kill_mission/mission_switch_msg", 10, &MissionSwitchStatus::updateStatus, this);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        ros::Rate r(10);
        r.sleep();
        ros::spinOnce();
        if (_mission_status)
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

private:
    void updateStatus(const std_msgs::Bool::ConstPtr &msg)
    {
        _mission_status = msg->data;
    }

    ros::NodeHandle _nh;
    ros::Subscriber _mission_switch_sub;
    bool _mission_status;
};