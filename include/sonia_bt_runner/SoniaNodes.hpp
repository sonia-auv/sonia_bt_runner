#pragma once

#include "sonia_bt_runner/vision/SoniaVision.hpp"
#include "sonia_bt_runner/navigation/SoniaNavigation.hpp"
#include "sonia_bt_runner/init/SoniaInit.hpp"

using namespace BT;

static void registerConditionNodes(BT::BehaviorTreeFactory &factory)
{
    
}
static void registerSyncActionNodes(BT::BehaviorTreeFactory &factory)
{
    
}

static void registerStatefulActionNodes(BT::BehaviorTreeFactory &factory)
{
    
}

static void registerNodes(BT::BehaviorTreeFactory &factory)
{
    registerConditionNodes(factory);
    registerSyncActionNodes(factory);
    registerStatefulActionNodes(factory);
}