#include "sonia_bt_runner/sync_actions/GetClosestObj.hpp"

GetClosestObj::GetClosestObj(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
{
}

BT::Nodestatus GetClosestObj::tick()
{
    AiDetectionArray aiArr;
    getInput<AiDetectionArray>("array", aiArr);

    AiDetection closest = aiArr.detection_array[0];

    for (size_t i = 1; i < aiArr.detection_array.size(); i++)
    {
        if (aiArr.detection_array[i].distance < closest.distance)
        {
            closest = aiArr.detection_array[i];
        }
    }
    setOutput("aiObj", closest);

    return BT::NodeStatus::SUCCESS;
}