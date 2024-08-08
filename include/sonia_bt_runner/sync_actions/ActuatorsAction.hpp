#pragma once
#include <chrono>
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common/ActuatorDoAction.h"
#include "sonia_common/ActuatorSendReply.h"
#include "ros/ros.h"

using namespace std::chrono_literals;

class ActuatorsAction : public BT::SyncActionNode
{
public:
    ActuatorsAction(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        // Options for 'side' are 'port_side' or 'starboard'
        return {BT::InputPort<int>("actuator", "0: Dropper, 1: Torpedo"),
                BT::InputPort<std::string>("side", "port_side or starboard")};
    }

    BT::NodeStatus tick() override;

private:
    void check_input(BT::Expected<std::string> side, BT::Expected<int> actuator);
    void actuator_callback(const sonia_common::ActuatorSendReply::ConstPtr &msg);

    ros::NodeHandle _nh;
    ros::Publisher _actuator_pub;
    ros::Subscriber _actuator_sub;
    sonia_common::ActuatorSendReply _reply_msg;
};