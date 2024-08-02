#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include <std_msgs/UInt8.h>
#include "sonia_common/MpcInfo.h"
#include <chrono>
#include "ros/ros.h"

class ControlSetMode: public BT::StatefulActionNode
{
    //This code must be tested
    public:
        ControlSetMode(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config),
            _timer(std::chrono::steady_clock::now()),
            _timeout(std::chrono::seconds(5))
        {    
            
            _setMode_pub= _nh.advertise<std_msgs::UInt8>("/proc_control/set_mode",100);
        }   
        ~ControlSetMode(){}
        
        static BT::PortsList providedPorts()
        {
            const bool mode= false;
            return {BT::InputPort<bool>("set_mode",mode, "")};
        }
        BT::NodeStatus onStart() override
        {
            std_msgs::UInt8 msg;
            bool data = false;
            getInput<bool>("set_mode", data);

            if(data)
            {
                msg.data =10;
                _setMode_pub.publish(msg);
            }
            else{
                msg.data=0;
                _setMode_pub.publish(msg);
            }
            _control_info_sub = _nh.subscribe("/proc_control/controller_info", 1, &ControlSetMode::control_callback, this);
            return BT::NodeStatus::RUNNING;
        }
        BT::NodeStatus onRunning() override{
            if(_ctrl_state==10 || _ctrl_state==0){
                ROS_INFO("control set mode");
                return BT::NodeStatus::SUCCESS;
            }
            auto elapsed_time=std::chrono::steady_clock::now()-_timer;
            if(elapsed_time >_timeout){
                ROS_WARN("time run out");
                return BT::NodeStatus::FAILURE;
            }      
            return BT::NodeStatus::RUNNING;
        }
        void onHalted() override {}
        
    private:
        void control_callback(const sonia_common::MpcInfo &msg){
            _ctrl_state= msg.mpc_mode;
        }
        ros::NodeHandle _nh;
        ros::Publisher _setMode_pub;
        ros::Subscriber _control_info_sub;

        uint8_t _ctrl_state;
        std::chrono::steady_clock::time_point _timer;
        std::chrono::steady_clock::duration _timeout;
};