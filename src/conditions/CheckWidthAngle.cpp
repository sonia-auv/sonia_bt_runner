#include "sonia_bt_runner/conditions/CheckWidthAngle.hpp"

CheckWidthAngle::CheckWidthAngle(const std::string &name, const BT::NodeConfig &config)
    : BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckWidthAngle::tick()
{
    AiDetection aiObj;
    getInput<AiDetection>("aiObj", aiObj);
    float min_ratio = 0.0;
    getInput<float>("minRatio", min_ratio);
    float max_ratio = 0.0;
    getInput<float>("maxRatio", max_ratio);

    float detected_ratio = (aiObj.left - aiObj.right) / (aiObj.top - aiObj.bottom);
    if (detected_ratio < min_ratio || detected_ratio > max_ratio)
    {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}