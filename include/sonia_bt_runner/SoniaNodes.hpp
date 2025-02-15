#pragma once

#include "sonia_bt_runner/vision/SoniaVision.hpp"
#include "sonia_bt_runner/navigation/SoniaNavigation.hpp"
#include "sonia_bt_runner/init/SoniaInit.hpp"

using namespace BT;

static void registerInitNodes(BT::BehaviorTreeFactory &factory)
{
    factory.registerNodeType<init::MissionSwitchStatus>("MissionSwitchStatus");
    factory.registerNodeType<init::Tare>("Tare");
    factory.registerNodeType<init::EnableDisableDVL>("EnableDisableDVL");
    factory.registerNodeType<init::ControlSetMode>("ControlSetMode");
}
static void registerNavigationNodes(BT::BehaviorTreeFactory &factory)
{
    
}

static void registerVisionNodes(BT::BehaviorTreeFactory &factory)
{
    
}

static void registerNodes(BT::BehaviorTreeFactory &factory)
{
    registerInitNodes(factory);
    registerNavigationNodes(factory);
    registerVisionNodes(factory);
}