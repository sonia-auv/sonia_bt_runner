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
    printf("NUM Targets found: %d\n", torpedoObjArr.detection_array.size());
    if (torpedoObjArr.detection_array.size() < 4)
    {
        printf("WARNING: DID NOT FIND 4 TARGETS. SIZE SELECTION MAY BE OFF.\n");
    }

    AiDetection obj1 = torpedoObjArr.detection_array[0];

    float cx1 = obj1.right - obj1.left;
    float cy1 = obj1.bottom - obj1.top;

    AiDetection obj2 = torpedoObjArr.detection_array[1];

    float cx2 = obj2.right - obj2.left;
    float cy2 = obj2.bottom - obj2.top;

    float diffx = abs(cx2 - cx1);
    float diffy = abs(cy2 - cy1);

    bool change_x = false;
    bool change_y = false;
    bool change_xy = false;

    if (diffx > diffy)
    {
        bool change_x_60 = false;
        if (diffx * 0.6 > diffy)
        {
            change_x_60 = true;
        }
        else
        {
            change_xy = true;
        }

        bool change_x_140 = false;
        if (diffx > diffy * 1.4)
        {
            change_x_140 = true;
        }
        else
        {
            change_xy = true
        }
        change_x = change_x_60 & change_x_140 && !change_xy;
    }

    if (diffy > diffx)
    {
        bool change_y_60 = false;
        if (diffy * 0.6 > diffz)
        {
            change_y_60 = true;
        }
        else
        {
            change_xy = true;
        }

        bool change_y_140 = false;
        if (diffy > diffx * 1.4)
        {
            change_y_140 = true;
        }
        else
        {
            change_xy = true
        }
        change_y = change_y_60 & change_y_140 && !change_xy;
    }

    if (change_x)
    {
        // Horizontal
    }
    else if (change_y)
    {
        // Vertical
    }
    else if (change_xy)
    {
        // diagonal
    }

    return BT::NodeStatus::FAILURE;
}