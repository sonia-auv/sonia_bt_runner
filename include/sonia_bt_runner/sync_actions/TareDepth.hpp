#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include "ros/ros.h"

class TareDepth : public BT::SyncActionNode
{
    // This code must be tested
public:
    TareDepth(const std::string &name)
        : BT::SyncActionNode(name, {})
    {
        _depth_client = _nh.serviceClient<std_srvs::Empty>("/provider_depth/tare");
    }
    ~TareDepth() {}
    BT::NodeStatus tick() override;

private:
    ros::NodeHandle _nh;
    ros::ServiceClient _depth_client;
};