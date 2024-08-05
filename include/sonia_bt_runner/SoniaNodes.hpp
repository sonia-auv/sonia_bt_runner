#pragma once

#include "sonia_bt_runner/conditions/SoniaConditions.hpp"
#include "sonia_bt_runner/sync_actions/SoniaSyncActions.hpp"
#include "sonia_bt_runner/stateful_actions/SoniaStatefulActions.hpp"

using namespace BT;

static void registerConditionNodes(BT::BehaviorTreeFactory &factory)
{
    factory.registerNodeType<MissionSwitchStatus>("MissionSwitchStatus");
}
static void registerSyncActionNodes(BT::BehaviorTreeFactory &factory)
{
    factory.registerNodeType<ActuatorsAction>("ActuatorsAction");
    factory.registerNodeType<InitTraj>("InitTraj");
    factory.registerNodeType<AddPoseToTraj>("AddPoseToTraj");
    factory.registerNodeType<AiActivation>("AiActivation");
    factory.registerNodeType<GetAiInfo>("GetAiInfo");
    factory.registerNodeType<EnableDisableDVL>("EnableDisableDVL");
    factory.registerNodeType<TareDepth>("TareDepth");
}

static void registerStatefulActionNodes(BT::BehaviorTreeFactory &factory)
{
    factory.registerNodeType<SendTrajToPlanner>("SendTrajToPlanner");
    factory.registerNodeType<WaitTargetReached>("WaitTargetReached");
    factory.registerNodeType<ControlSetMode>("ControlSetMode");
}

static void registerNodes(BT::BehaviorTreeFactory &factory)
{
    registerConditionNodes(factory);
    registerSyncActionNodes(factory);
    registerStatefulActionNodes(factory);
}