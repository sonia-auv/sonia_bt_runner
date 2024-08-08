#include "sonia_bt_runner/sync_actions/MoveToTarget.hpp"

MoveToTarget::MoveToTarget(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus MoveToTarget::tick()
{
    AiDetection aiObj;
    getInput<AiDetection>("aiObj", aiObj);

    float offset;
    getInput<float>("offset", offset);

    TrajectoryPose tp;
    tp.positionX = aiObj.distance + offset;
    tp.positionY = 0;
    tp.positionZ = 0;
    tp.orientationX = 0;
    tp.orientationY = 0;
    tp.orientationZ = 0;
    tp.frame = 1;
    tp.speed = 0;
    tp.precision = 0;
    tp.long_rotation = false;

    Trajectory t;
    getInput<Trajectory>("trajectory", t);
    t.trajectory.push_back(tp);
    setOutput("trajectory", t);

    return BT::NodeStatus::SUCCESS;
}