#pragma once

#include "sonia_bt_runner/vision/SoniaVision.hpp"
#include "sonia_bt_runner/navigation/SoniaNavigation.hpp"
#include "sonia_bt_runner/init/SoniaInit.hpp"
#include "sonia_bt_runner/actuator/SoniaActuator.hpp"

using namespace BT;
template <typename T>
BT::NodeBuilder configBuilder(std::shared_ptr<rclcpp::Node> node);

template <typename T>
BT::NodeBuilder simpleBuilder(std::shared_ptr<rclcpp::Node> node);

static void registerInitNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<rclcpp::Node> node)
{
    factory.registerBuilder<init::MissionSwitchStatus>("MissionSwitchStatus", simpleBuilder<init::MissionSwitchStatus>(node));
    factory.registerBuilder<init::EnableDisableDVL>("EnableDisableDVL", configBuilder<init::EnableDisableDVL>(node));
    factory.registerBuilder<init::ControlSetMode>("ControlSetMode", configBuilder<init::ControlSetMode>(node));
    factory.registerBuilder<init::Tare>("Tare", configBuilder<init::Tare>(node));     
}
static void registerActuatorNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<rclcpp::Node> node)
{
    factory.registerBuilder<actuator::Dropper>("Droppers", configBuilder<actuator::Dropper>(node));
    factory.registerBuilder<actuator::Torpedo>("Torpedos", configBuilder<actuator::Torpedo>(node)); 
}
static void registerNavigationNodes(BT::BehaviorTreeFactory &factory)
{
    //factory.registerNodeType<navigation::SendWaypoint>("SendWaypoint");
}

static void registerVisionNodes(BT::BehaviorTreeFactory &factory)
{
    
}

static void registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<rclcpp::Node> node)
{
    registerInitNodes(factory, node);
    registerActuatorNodes(factory, node);
   //registerNavigationNodes(factory);
    //registerVisionNodes(factory);
}

template <typename T>
BT::NodeBuilder configBuilder(std::shared_ptr<rclcpp::Node> node){
    return [node](const std::string &name, const BT::NodeConfig &config){
        return std::make_unique<T>(name,config,node);
    }; 
}

template <typename T>
BT::NodeBuilder simpleBuilder(std::shared_ptr<rclcpp::Node> node){
    return [node](const std::string &name, const BT::NodeConfig &config){
        return std::make_unique<T>(name,node);
    }; 
}