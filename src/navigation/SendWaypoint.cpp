#include "sonia_bt_runner/navigation/SendWaypoint.hpp"

namespace navigation{
    SendWaypoint::SendWaypoint(const std::string& name, const BT::NodeConfig& config) 
    : BT::StatefulActionNode(name, config), ros_node{std::make_shared<rclcpp::Node>(name)}
    {

    }
    SendWaypoint::~SendWaypoint(){}

    BT::NodeStatus SendWaypoint::onStart(){

    }
    BT::NodeStatus SendWaypoint::onRunning(){

    }
    void SendWaypoint::onHalted(){}
}
