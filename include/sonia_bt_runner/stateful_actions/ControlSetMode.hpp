#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include <std_msgs/UInt8.h>
#include "sonia_common/MpcInfo.h"
#include <chrono>
#include "ros/ros.h"

class ControlSetMode : public BT::StatefulActionNode
{
    // This code must be tested
public:
    ControlSetMode(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config),
          _timer(std::chrono::steady_clock::now()),
          _timeout(std::chrono::seconds(5))
    {

        _setMode_pub = _nh.advertise<std_msgs::UInt8>("/proc_control/set_mode", 100);
    }
    ~ControlSetMode() {}

    static BT::PortsList providedPorts()
    {
        const bool mode = false;
        return {BT::InputPort<bool>("set_mode", mode, "")};
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:
    void control_callback(const sonia_common::MpcInfo &msg);

    ros::NodeHandle _nh;
    ros::Publisher _setMode_pub;
    ros::Subscriber _control_info_sub;

    uint8_t _ctrl_state;
    std::chrono::steady_clock::time_point _timer;
    std::chrono::steady_clock::duration _timeout;
};