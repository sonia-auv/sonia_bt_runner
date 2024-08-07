#include "sonia_bt_runner/conditions/CheckTranslationAlign.hpp"
#include "sonia_bt_runner/utils/TrajectoryPose.hpp"

CheckTranslationAlign::CheckTranslationAlign(const std::string name, const BT::NodeConfig &config)
    : BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckTranslationAlign::tick()
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
    // Check if object is near the edge of the screen
    bool half_width = false;
    if (x1 < 20 || x2 > _screen_width - 20)
    {
        half_width = true;
    }

    bool half_height = false;
    if (y1 < 20 || y2 > _screen_height - 20)
    {
        half_height = true;
    }

    int target_x = 0;
    getInput<int>("target_x", target_x);
    int target_y = 0;
    getInput<int>("target_y", target_y);
    float error_marg = 0.0;
    getInput<float>("error_margin", error_marg);

    float diff_x = cx - target_x;
    float diff_y;
    if (target_y == -1)
    {
        diff_y = 0;
    }
    else
    {
        diff_y = cy - target_y;
    }
    printf("diffx: %.2f\n", diff_x);
    printf("diffy: %.2f\n", diff_y);
    printf("dist: %.2f\n", aiObj.distance);

    if (abs(diff_x) <= target_x * error_marg)
    {
        diff_x = 0;
    }

    if (abs(diff_y) <= target_y * error_marg)
    {
        diff_y = 0;
    }

    if (diff_x == 0 and diff_y == 0)
    {
        return BT::NodeStatus::SUCCESS;
    }

    TrajectoryPose tp;
    tp.frame = 1;
    tp.long_rotation = false;
    tp.precision = 0;
    tp.speed = 0;
    tp.positionX = 0;
    tp.orientationX = 0;
    tp.orientationY = 0;
    tp.orientationZ = 0;
    tp.positionY = diff_x * aiObj.distance * _pixel_to_meters;
    tp.positionZ = diff_y * aiObj.distance * _pixel_to_meters;
    // printf("%.2f\n", tp.positionZ);
    // tp.positionZ = 0;
    if (half_width)
    {
        tp.positionY = tp.positionY / 2;
    }
    if (half_height)
    {
        tp.positionZ = tp.positionZ / 2;
    }

    Trajectory t;
    getInput<Trajectory>("traj", t);
    t.trajectory.push_back(tp);

    setOutput("traj", t);
    return BT::NodeStatus::FAILURE;
}