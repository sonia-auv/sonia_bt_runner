#pragma once

#include "sonia_bt_runner/vision/SoniaVision.hpp"
#include "sonia_bt_runner/navigation/SoniaNavigation.hpp"
#include "sonia_bt_runner/init/SoniaInit.hpp"

using namespace BT;
template <typename T>
static BT::NodeBuilder builder1(std::shared_ptr<rclcpp::Node> node);

template <typename T>
static BT::NodeBuilder builder2(std::shared_ptr<rclcpp::Node> node);

static void registerInitNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<rclcpp::Node> node)
{
    factory.registerBuilder<init::MissionSwitchStatus>("MissionSwitchStatus", builder2<init::MissionSwitchStatus>(node));
    //factory.registerBuilder<init::EnableDisableDVL>("EnableDisableDVL", builder<init::EnableDisableDVL>(node));
    //factory.registerNodeType<init::Tare>("Tare");
    //factory.registerNodeType<init::EnableDisableDVL>("EnableDisableDVL");
    //factory.registerNodeType<init::ControlSetMode>("ControlSetMode");
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
   //registerNavigationNodes(factory);
    //registerVisionNodes(factory);
}

template <typename T>
static BT::NodeBuilder builder1(std::shared_ptr<rclcpp::Node> node){
    return [node](const std::string &name, const BT::NodeConfig &config){
        return std::make_unique<T>(name,config,node);
    }; 
}

template <typename T>
static BT::NodeBuilder builder2(std::shared_ptr<rclcpp::Node> node){
    return [node](const std::string &name, const BT::NodeConfig &config){
        return std::make_unique<T>(name,node);
    }; 
}