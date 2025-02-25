#include "sonia_bt_runner/mechanism/Actuator.hpp"

namespace mechanism{
    Actuator::Actuator(const std::string &name, const BT::NodeConfig &config)
    :BT::StatefulActionNode(name, config), ros_node{std::make_shared<rclcpp::Node>(name)}
    {

    }
    Actuator::~Actuator(){}

    BT::NodeStatus Actuator::onStart(){

    }
    BT::NodeStatus Actuator::onRunning(){

    }
    void Actuator::onHalted(){}

}