#include "sonia_bt_runner/stateful_actions/ControlSetMode.hpp"

ControlSetMode::ControlSetMode(const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config),
      _timer(std::chrono::steady_clock::now()),
      _timeout(std::chrono::seconds(5))
{

    _setMode_pub = _nh.advertise<std_msgs::UInt8>("/proc_control/set_mode", 100);
}

BT::NodeStatus ControlSetMode::onStart()
{
    std_msgs::UInt8 msg;
    bool data = false;
    getInput<bool>("set_mode", data);

    if (data)
    {
        msg.data = 10;
        _setMode_pub.publish(msg);
    }
    else
    {
        msg.data = 0;
        _setMode_pub.publish(msg);
    }
    _control_info_sub = _nh.subscribe("/proc_control/controller_info", 1, &ControlSetMode::control_callback, this);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ControlSetMode::onRunning()
{
    if (_ctrl_state == 10 || _ctrl_state == 0)
    {
        ROS_INFO("control set mode");
        return BT::NodeStatus::SUCCESS;
    }
    auto elapsed_time = std::chrono::steady_clock::now() - _timer;
    if (elapsed_time > _timeout)
    {
        ROS_WARN("time run out");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

void ControlSetMode::onHalted()
{
}

void ControlSetMode::control_callback(const sonia_common::MpcInfo &msg)
{
    _ctrl_state = msg.mpc_mode;
}