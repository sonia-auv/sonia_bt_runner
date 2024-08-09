#include "sonia_bt_runner/conditions/CheckRotationZAlign.hpp"
#include "sonia_bt_runner/utils/TrajectoryPose.hpp"

CheckRotationZAlign::CheckRotationZAlign(const std::string name, const BT::NodeConfig &config)
    : BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckRotationZAlign::tick()
{
    AiDetection aiObj;
    getInput<AiDetection>("detection_object", aiObj);
    float x1 = aiObj.left;
    float x2 = aiObj.right;
    float y1 = aiObj.top;
    float y2 = aiObj.bottom;
    float cx = x1 + ((x2 - x1) / 2);
    float cy = y1 + ((y2 - y1) / 2);
    printf("cx: %.2f\n", cx);
    printf("cy: %.2f\n", cy);

    int target_x = 0;
    getInput<int>("target_x", target_x);
    float error_marg = 0.0;
    getInput<float>("error_margin", error_marg);

    float diff_x = cx - target_x;

    if (abs(diff_x) <= target_x * error_marg)
    {
        return BT::NodeStatus::SUCCESS;
    }

    float angle_degree = diff_x * _pixel_to_degrees;

    printf("diffx: %.2f\n", diff_x);
    printf("angle: %.2f\n", angle_degree);    

    TrajectoryPose tp;
    tp.frame = 1;
    tp.long_rotation = false;
    tp.precision = 0;
    tp.speed = 0;
    tp.orientationX = 0;
    tp.orientationY = 0;
    tp.orientationZ = angle_degree;
    tp.positionX = 0;
    tp.positionY = 0;
    tp.positionZ = 0;

    Trajectory t;
    getInput<Trajectory>("traj", t);
    t.trajectory.push_back(tp);

    setOutput("traj", t);
    return BT::NodeStatus::FAILURE;
}