#pragma once

#include "sonia_bt_runner/conditions/SoniaConditions.hpp"
#include "sonia_bt_runner/sync_actions/SoniaSyncActions.hpp"
#include "sonia_bt_runner/stateful_actions/SoniaStatefulActions.hpp"

using namespace BT;

static void registerConditionNodes(BT::BehaviorTreeFactory &factory)
{
    factory.registerNodeType<CheckAiInfo>("CheckAiInfo");
    factory.registerNodeType<CheckRotationZAlign>("CheckRotationZAlign");
    factory.registerNodeType<CheckTranslationXYAlign>("CheckTranslationXYAlign");
    factory.registerNodeType<CheckTranslationYZAlign>("CheckTranslationYZAlign");
    factory.registerNodeType<MissionSwitchStatus>("MissionSwitchStatus");
}

static void registerStatefulActionNodes(BT::BehaviorTreeFactory &factory)
{
    factory.registerNodeType<AverageAiDetection>("AverageAiDetection");
    factory.registerNodeType<ControlSetMode>("ControlSetMode");
    factory.registerNodeType<SendTrajToPlanner>("SendTrajToPlanner");
    factory.registerNodeType<WaitTargetReached>("WaitTargetReached");
}

static void registerSyncActionNodes(BT::BehaviorTreeFactory &factory)
{
    factory.registerNodeType<ActuatorsAction>("ActuatorsAction");
    factory.registerNodeType<AddPoseToTraj>("AddPoseToTraj");
    factory.registerNodeType<AiActivation>("AiActivation");
    factory.registerNodeType<EnableDisableDVL>("EnableDisableDVL");
    factory.registerNodeType<ExtractAiListItem>("ExtractAiListItem");
    factory.registerNodeType<GetAngle>("GetAngle");
    factory.registerNodeType<GetClosestObj>("GetClosestObj");
    factory.registerNodeType<GetTorpedoTarget>("GetTorpedoTarget");
    factory.registerNodeType<InitTraj>("InitTraj");
    factory.registerNodeType<MoveToTarget>("MoveToTarget");
    factory.registerNodeType<TareDepth>("TareDepth");
}

static void registerNodes(BT::BehaviorTreeFactory &factory)
{
    registerConditionNodes(factory);
    registerSyncActionNodes(factory);
    registerStatefulActionNodes(factory);
}