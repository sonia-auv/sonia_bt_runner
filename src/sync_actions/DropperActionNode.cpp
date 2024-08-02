#include "sonia_bt_runner/sync_actions/DropperActionNode.hpp"

BT::NodeStatus DropperActionNode::tick()
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

void DropperActionNode::dropper_callback(const sonia_common::ActuatorSendReply::ConstPtr &msg)
{
    _reply_msg.element = msg->element;
    _reply_msg.side = msg->side;
    _reply_msg.response = msg->response;
}
