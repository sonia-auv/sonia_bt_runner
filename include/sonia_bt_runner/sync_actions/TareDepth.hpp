#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include "ros/ros.h"

class TareDepth: public BT::SyncActionNode
{
    //This code must be tested
    public:
        TareDepth(const std::string &name, const BT::NodeConfig &config)
            : BT::SyncActionNode(name, config)
        {    
            _depth_client = _nh.serviceClient<std_srvs::Empty>("/provider_depth/tare");   
        }   
        ~TareDepth(){}
         BT::NodeStatus tick() override
        {
            ros::Rate r(0.25);
            std_srvs::Empty tare;

            if(_depth_client.call(tare)){
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

    private:
        ros::NodeHandle _nh;
        ros::ServiceClient _depth_client;
};