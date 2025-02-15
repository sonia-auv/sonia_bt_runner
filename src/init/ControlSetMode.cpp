#include "sonia_bt_runner/init/ControlSetMode.hpp"

namespace init{
    ControlSetMode::ControlSetMode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), ros_node{std::make_shared<rclcpp::Node>(name)}
    {

    }
    ControlSetMode::~ControlSetMode(){}

    BT::NodeStatus ControlSetMode::onStart(){

    }
    BT::NodeStatus ControlSetMode::onRunning(){

    }
    void ControlSetMode::onHalted(){
    }
    void ControlSetMode::control_callback(const sonia_common_ros2::msg::MpcInfo &msg){

    }

}