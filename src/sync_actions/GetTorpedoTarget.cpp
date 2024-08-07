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
    if (torpedoObjArr.detection_array.size() < 4)
    {
        printf("WARNING: DID NOT FIND 4 TARGETS. SIZE SELECTION MAY BE OFF.\n");
    }

    AiDetection obj1 = torpedoObjArr.detection_array[0];

    float cx1 = obj1.left + (obj1.right - obj1.left) / 2;
    float cy1 = obj1.top + (obj1.bottom - obj1.top) / 2;

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
            change_xy = true;
        }
        change_x = change_x_60 & change_x_140 && !change_xy;
    }

    if (diffy > diffx)
    {
        bool change_y_60 = false;
        if (diffy * 0.6 > diffx)
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
            change_xy = true;
        }
        change_y = change_y_60 & change_y_140 && !change_xy;
    }

    if (change_x)
    {
        // Horizontal
        AiDetection left_target;
        AiDetection right_target;
        if (cx1 < cx2)
        {
            left_target = obj1;
            right_target = obj2;
        }
        else
        {
            left_target = obj2;
            right_target = obj1;
        }
        if (get_area(left_target) < get_area(right_target))
        {
            // top row
        }
        else
        {
            // bottow row
        }
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

float GetTorpedoTarget::get_area(AiDetection obj)
{
    float height = left_target.bottom - left_target.top;
    float width = left_target.right - left_target.left return height * width;
}