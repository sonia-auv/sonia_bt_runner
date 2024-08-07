#pragma once

#include "sonia_bt_runner/conditions/SoniaConditions.hpp"
#include "sonia_bt_runner/sync_actions/SoniaSyncActions.hpp"
#include "sonia_bt_runner/stateful_actions/SoniaStatefulActions.hpp"

using namespace BT;

static void registerConditionNodes(BT::BehaviorTreeFactory &factory)
{
    factory.registerNodeType<MissionSwitchStatus>("MissionSwitchStatus");
    factory.registerNodeType<CheckAiInfo>("CheckAiInfo");
    factory.registerNodeType<CheckTranslationYZAlign>("CheckTranslationYZAlign");
    factory.registerNodeType<CheckTranslationXYAlign>("CheckTranslationXYAlign");

}

static void registerSyncActionNodes(BT::BehaviorTreeFactory &factory)
{
    factory.registerNodeType<ActuatorsAction>("ActuatorsAction");
    factory.registerNodeType<InitTraj>("InitTraj");
    factory.registerNodeType<AddPoseToTraj>("AddPoseToTraj");
    factory.registerNodeType<AiActivation>("AiActivation");
    factory.registerNodeType<EnableDisableDVL>("EnableDisableDVL");
    factory.registerNodeType<TareDepth>("TareDepth");
    factory.registerNodeType<MoveToTarget>("MoveToTarget");
    factory.registerNodeType<GetAngle>("GetAngle");
    factory.registerNodeType<GetTorpedoTarget>("GetTorpedoTarget");
    factory.registerNodeType<ExtractAiListItem>("ExtractAiListItem");
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