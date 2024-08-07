#pragma once
#include <string.h>
#include <stdio.h>
#include "behaviortree_cpp/behavior_tree.h"
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include "ros/ros.h"

class TareDepth : public BT::SyncActionNode
{
    // This code must be tested
public:
    TareDepth(const std::string &name);

    BT::NodeStatus tick() override;

private:
    ros::NodeHandle _nh;
    ros::ServiceClient _depth_client;
    const char *auv;
};