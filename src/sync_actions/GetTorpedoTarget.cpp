#include "sonia_bt_runner/sync_actions/GetTorpedoTarget.hpp"
#include "sonia_bt_runner/utils/AiDetection.hpp"

GetTorpedoTarget::GetTorpedoTarget(const std::string name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GetTorpedoTarget::tick()
{
    AiDetectionArray torpedoObjArr;
    getInput<AiDetectionArray>("torpedo_objs", torpedoObjArr);
    printf("NUM Targets found: %ld\n", torpedoObjArr.detection_array.size());
    if (torpedoObjArr.detection_array.size() < 2)
    {
        return BT::NodeStatus::FAILURE;
    }
    if (torpedoObjArr.detection_array.size() < 4)
    {
        printf("WARNING: DID NOT FIND 4 TARGETS. SIZE SELECTION MAY BE OFF.\n");
    }

    std::vector<AiDetection> targets_in_order{torpedoObjArr.detection_array[0]};
    for (size_t i = 1; i < torpedoObjArr.detection_array.size(); i++)
    {
        AiDetection aiObj = torpedoObjArr.detection_array[i];
        float area = get_area(aiObj);
        bool is_smaller = false;
        for (size_t j = 0; j < targets_in_order.size(); j++)
        {
            if (area < get_area(targets_in_order[j]))
            {
                targets_in_order.insert(targets_in_order.begin() + j, aiObj);
                is_smaller = true;
                break;
            }
        }
        if (!is_smaller)
        {
            targets_in_order.push_back(aiObj);
        }
    }

    int index = 0;
    getInput("size", index);
    setOutput("selected_target", targets_in_order[index]);

    return BT::NodeStatus::SUCCESS;
}

float GetTorpedoTarget::get_area(AiDetection obj)
{
    float height = obj.bottom - obj.top;
    float width = obj.right - obj.left;
    return height * width;
}