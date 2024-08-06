#include "sonia_bt_runner/sync_actions/GetAngle.hpp"
#include <math.h>

GetAngle::GetAngle(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GetAngle::tick()
{
    AiDetection aiObj;
    getInput<AiDetection>("aiObj", aiObj);
    float ideal_ratio = 0.0;
    getInput<float>("ideal_ratio", ideal_ratio);
    float error_marg = 0.0;
    getInput<float>("error_margin", error_marg);

    float height = aiObj.top - aiObj.bottom;
    float seen_width = aiObj.right - aiObj.left;
    float real_width = height * ideal_ratio;

    float angle = acos(seen_width/real_width);
    setOutput("detected_angle", angle);
    return BT::NodeStatus::SUCCESS;
}