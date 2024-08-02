#pragma once
#include <chrono>
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common/ActuatorDoAction.h"
#include "sonia_common/ActuatorSendReply.h"
#include "ros/ros.h"

using namespace std::chrono_literals;

class DropperActionNode : public BT::SyncActionNode
{
public:
    DropperActionNode(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)
    {
        _dropper_pub = _nh.advertise<sonia_common::ActuatorDoAction>("/provider_actuators/do_action_to_actuators", 100);
        _dropper_sub = _nh.subscribe("/provider_actuators/do_action_from_actuators", 10, &DropperActionNode::dropper_callback, this);
    }

    ~DropperActionNode() {};

    static BT::PortsList providedPorts()
    {
        // Options for 'side' are 'port_side' or 'starboard'
        return {BT::InputPort<std::string>("side")};
    }

    BT::NodeStatus tick() override;

private:
    void dropper_callback(const sonia_common::ActuatorSendReply::ConstPtr &msg);

    ros::NodeHandle _nh;
    ros::Publisher _dropper_pub;
    ros::Subscriber _dropper_sub;
    sonia_common::ActuatorSendReply _reply_msg;
};