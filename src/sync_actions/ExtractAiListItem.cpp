#include "sonia_bt_runner/sync_actions/ExtractAiListItem.hpp"

ExtractAiListItem::ExtractAiListItem(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus ExtractAiListItem::tick()
{
    int index = 0;
    getInput<int>("index", index);
    AiDetectionArray arr;
    getInput<AiDetectionArray>("array", arr);
    if (index >= arr.detection_array.size())
    {
        return BT::NodeStatus::FAILURE;
    }

    setOutput("aiObject", arr.detection_array[index]);
    return BT::NodeStatus::SUCCESS;
}