#pragma once

#include "sonia_bt_runner/vision/SoniaVision.hpp"
#include "sonia_bt_runner/navigation/SoniaNavigation.hpp"
#include "sonia_bt_runner/init/SoniaInit.hpp"

using namespace BT;

static void registerInitNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<rclcpp::Node> node)
{
    //factory.registerNodeType<init::MissionSwitchStatus>("MissionSwitchStatus");
    factory.registerBuilder<init::MissionSwitchStatus>("MissionSwitchStatus",
    [node](const std::string &name, const BT::NodeConfig &config){
        return std::make_unique<init::MissionSwitchStatus>(name,node);
    });
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