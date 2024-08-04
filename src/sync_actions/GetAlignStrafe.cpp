#include "sonia_bt_runner/sync_actions/GetAlignStrafe.hpp"

#include "sonia_bt_runner/utils/TrajectoryPose.hpp"


GetAlignStrafe::GetAlignStrafe(const std::string name,const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GetAlignStrafe::tick()
{
    AiDetection aiObj;
    getInput<AiDetection>("detection_object", aiObj);
    float x1 = aiObj.left;
    float x2 = aiObj.right;
    float y1 = aiObj.top;
    float y2 = aiObj.bottom;
    float cx = x1 + ((x2 - x1) / 2);
    float cy = y1 + ((y2 - y1) / 2);

    int target_x = 0;
    getInput<int>("target_x", target_x);
    int target_y = 0;
    getInput<int>("target_x", target_y);

    float diff_x = target_x - cx;
    float diff_y = target_y - cy;

    TrajectoryPose tp;

    tp.positionY = diff_x * aiObj.distance * _pixel_to_meters;
    tp.positionZ = diff_y * aiObj.distance * _pixel_to_meters;

    Trajectory t;
    getInput<Trajectory>("traj", t);
    t.trajectory.push_back(tp);

    setOutput("traj", t);
    return BT::NodeStatus::SUCCESS;
}