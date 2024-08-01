#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include "ros/ros.h"

class EnableDisableDVL: public BT::SyncActionNode
{
    //This code must be tested
    public:
        EnableDisableDVL(const std::string &name, const BT::NodeConfig &config)
            : BT::SyncActionNode(name, config)
        {    
            _dvl_pub= _nh.advertise<std_msgs::Bool>("/provider_dvl/enable_disable_ping",100);   
        }   
        
        static BT::PortsList providedPorts()
        {
            const bool  dvl_state= true;
            return {BT::InputPort<bool>("Toggle_DVl", dvl_state, "")};
        }
        BT::NodeStatus tick() override
        {
            ros::Rate r(0.25);
            std_msgs::Bool dvl_state;

            bool data = false;
            getInput<bool>("Toggle_DVL", data);
            if(data){
                ROS_INFO("DVL is started");
            }
            else{
                ROS_INFO("DVL is stopped");
            }
            dvl_state.data=data;
            _dvl_pub.publish(dvl_state);
            return BT::NodeStatus::SUCCESS;
        }
        

    private:
        ros::NodeHandle _nh;
        ros::Publisher _dvl_pub;
};