#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include <std_msgs/Bool.h>
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
        ~EnableDisableDVL(){}
        
        static BT::PortsList providedPorts()
        {
            const bool dvl_input= true;
            return {BT::InputPort<bool>("dvl_input",dvl_input, "")};
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
            else if(!data){
                ROS_INFO("DVL is stopped");
            }
            else{
                ROS_INFO("Invalid input. input is boolean");
            }
            dvl_state.data=data;
            _dvl_pub.publish(dvl_state);
            return BT::NodeStatus::SUCCESS;
        }
        
    private:
        ros::NodeHandle _nh;
        ros::Publisher _dvl_pub;
};