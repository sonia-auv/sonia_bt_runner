#pragma once

#include "sonia_bt_runner/vision/SoniaVision.hpp"
#include "sonia_bt_runner/navigation/SoniaNavigation.hpp"
#include "sonia_bt_runner/init/SoniaInit.hpp"
#include "sonia_bt_runner/actuator/SoniaActuator.hpp"
#include "sonia_bt_runner/sensors/DepthCheck.hpp"

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
    factory.registerBuilder<actuator::Dropper>("Dropper", configBuilder<actuator::Dropper>(node));
    factory.registerBuilder<actuator::Torpedo>("Torpedo", configBuilder<actuator::Torpedo>(node)); 
}
static void registerNavigationNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<rclcpp::Node> node)
{
    factory.registerBuilder<navigation::Alignment>("Alignment", configBuilder<navigation::Alignment>(node));
    factory.registerNodeType<navigation::InitializeTrajectory>("InitializeTrajectory");
    factory.registerNodeType<navigation::TrajectoryAppendPose>("TrajectoryAppendPose");
    factory.registerBuilder<navigation::SendTrajectory>("SendTrajectory", configBuilder<navigation::SendTrajectory>(node));
    factory.registerBuilder<navigation::WaitTargetReached>("WaitTargetReached", configBuilder<navigation::WaitTargetReached>(node));
}

static void registerVisionNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<rclcpp::Node> node)
{
    factory.registerBuilder<vision::CheckDistance>("CheckDistance", configBuilder<vision::CheckDistance>(node));
    factory.registerBuilder<vision::AiActivation>("AiActivation", configBuilder<vision::AiActivation>(node));
    factory.registerBuilder<vision::AiFilter>("AiFilter", configBuilder<vision::AiFilter>(node));
}

static void registerSensorNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<rclcpp::Node> node)
{
    factory.registerBuilder<sensors::DepthCheck>("DepthCheck", simpleBuilder<sensors::DepthCheck>(node));
}

static void registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<rclcpp::Node> node)
{
    registerInitNodes(factory, node);
    registerActuatorNodes(factory, node);
    registerNavigationNodes(factory, node);
    registerVisionNodes(factory, node);
    registerSensorNodes(factory, node);
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