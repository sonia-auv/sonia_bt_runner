#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "ros/ros.h"
#include "sonia_common/MissionTimer.h"
#include "sonia_common/MpcInfo.h"

class WaitTargetReached : public BT::StatefulActionNode
{
public:
    WaitTargetReached(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config), _ros_rate(5)
    {
        _trajectory_done_prev = true;
        _traj_complete = false;
        _launch_time = std::chrono::system_clock::now();
        _param_timeout = 0;
        getInput<float>("timeout", _param_timeout);
        _timeout_pub = _nh.advertise<sonia_common::MissionTimer>("/sonia_behaviors/timeout", 5);
    }

    ~WaitTargetReached()
    {
        _controller_info_sub.shutdown();
    }

    static BT::PortsList providedPorts()
    {
        const int def_timeout = 5;
        return {
            BT::InputPort<int>("timeout", def_timeout, "")};
    }

    BT::NodeStatus onStart() override
    {
        ROS_INFO("Starting WaitTargetReached");
        _traj_complete = false;
        _time_diff = 0;
        _target_reached = false;
        _trajectory_done = true;
        _is_alive = true;

        _controller_info_sub = _nh.subscribe("/proc_control/controller_info", 1, &WaitTargetReached::get_controller_info_cb, this);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!_is_alive)
        {
            return BT::NodeStatus::FAILURE;
        }

        if (_traj_complete)
        {
            ROS_INFO("Traj Complete");
            _time_diff = (std::chrono::system_clock::now() - _launch_time).count();
        }

        if (_time_diff > _param_timeout || _target_reached)
        {
            ROS_INFO("Target reached status, %d", (int)_target_reached);
            if (_target_reached)
            {
                _timeout_pub.publish(missionTimerFunc("wait_target_reached", _param_timeout, std::chrono::system_clock::to_time_t(_launch_time), 2));
                ROS_INFO("Target Reached");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                _timeout_pub.publish(missionTimerFunc("wait_target_reached", _param_timeout, std::chrono::system_clock::to_time_t(_launch_time), 3));
                ROS_INFO("Target couldn't be reached");
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    void get_controller_info_cb(const sonia_common::MpcInfo &msg)
    {
        _target_reached = msg.target_reached;
        _trajectory_done = msg.is_trajectory_done;
        _is_alive = msg.is_mpc_alive;

        if (_trajectory_done != _trajectory_done_prev)
        {
            if (_trajectory_done)
            {
                _launch_time = std::chrono::system_clock::now();
                _traj_complete = true;
                _timeout_pub.publish(missionTimerFunc("wait_target_reached", _param_timeout, std::chrono::system_clock::to_time_t(_launch_time), 1));
                ROS_INFO("Trajectory Completed");
            }
            else
            {
                ROS_INFO("Trajectory has been received");
            }
        }
        _trajectory_done_prev = _trajectory_done;
    }

    static sonia_common::MissionTimer missionTimerFunc(std::string mission, int timeout, time_t uniqueID, int status)
    {
        sonia_common::MissionTimer buffer;
        buffer.mission = mission;
        buffer.timeout = timeout;
        buffer.uniqueID = std::to_string(uniqueID);
        buffer.status = status;
        return buffer;
    }

    ros::NodeHandle _nh;
    ros::Publisher _timeout_pub;
    ros::Subscriber _controller_info_sub;
    ros::Rate _ros_rate;

    std::chrono::_V2::system_clock::time_point _launch_time;

    bool _traj_complete;
    int _time_diff;
    bool _target_reached;
    bool _trajectory_done_prev;
    bool _trajectory_done;
    bool _is_alive;
    float _param_timeout;
};