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

    BT::NodeStatus tick() override
    {
        ros::Rate r(0.25);

        sonia_common::ActuatorDoAction msg;
        msg.element = sonia_common::ActuatorDoAction::ELEMENT_DROPPER;
        msg.action = sonia_common::ActuatorDoAction::ACTION_DROPPER_LAUNCH;
        BT::Expected<std::string> side = getInput<std::string>("side");
        if (!side)
        {
            throw BT::RuntimeError("missing required input [side]: ",
                                   side.error());
        }
        if (side.value() == "port_side")
        {
            msg.side = sonia_common::ActuatorDoAction::SIDE_PORT;
        }
        else if (side.value() == "starboard")
        {
            msg.side = sonia_common::ActuatorDoAction::SIDE_STARBOARD;
        }
        else
        {
            throw BT::RuntimeError("Bad required input [side]: ",
                                   side.error());
        }
        _dropper_pub.publish(msg);

        r.sleep();
        ros::spinOnce();
        if (_reply_msg.response == sonia_common::ActuatorSendReply::RESPONSE_SUCCESS)
        {
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    void dropper_callback(const sonia_common::ActuatorSendReply::ConstPtr &msg)
    {
        _reply_msg.element = msg->element;
        _reply_msg.side = msg->side;
        _reply_msg.response = msg->response;
    }

    ros::NodeHandle _nh;
    ros::Publisher _dropper_pub;
    ros::Subscriber _dropper_sub;
    sonia_common::ActuatorSendReply _reply_msg;
};