#include "sonia_bt_runner/sync_actions/GetAngle.hpp"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include <math.h>

GetAngle::GetAngle(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GetAngle::tick()
{
    AiDetectionArray aiObjarr;
    getInput<AiDetectionArray>("aiObj", aiObjarr);
    AiDetection aiObj = aiObjarr.detection_array[0];
    float ideal_ratio = 0.0;
    getInput<float>("ideal_ratio", ideal_ratio);
    float error_marg = 0.0;
    getInput<float>("error_margin", error_marg);

    float height = aiObj.bottom - aiObj.top;
    float seen_width = aiObj.right - aiObj.left;
    float real_width = height * ideal_ratio;

    if (seen_width <= real_width*(1-error_marg))
    {
        return BT::NodeStatus::SUCCESS;
    }
    printf("seen_width: %.2f\n", seen_width);
    printf("real_width: %.2f\n", real_width);
    if (seen_width > real_width){
        seen_width = real_width - (seen_width - real_width);
    }
    float angle = acos(seen_width/real_width);
    printf("angle: %.2f\n", angle);
    setOutput("detected_angle", angle);
    return BT::NodeStatus::FAILURE;
}