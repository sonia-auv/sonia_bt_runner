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
    DropperActionNode(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        // Options for 'side' are 'port_side' or 'starboard'
        return {BT::InputPort<std::string>("side", "port_side or starboard")};
    }

    BT::NodeStatus tick() override;

private:
    void dropper_callback(const sonia_common::ActuatorSendReply::ConstPtr &msg);

    ros::NodeHandle _nh;
    ros::Publisher _dropper_pub;
    ros::Subscriber _dropper_sub;
    sonia_common::ActuatorSendReply _reply_msg;
};