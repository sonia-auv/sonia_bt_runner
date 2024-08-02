#include "sonia_bt_runner/stateful_actions/WaitTargetReached.hpp"

BT::NodeStatus WaitTargetReached::onStart()
{
    ROS_INFO("Starting WaitTargetReached");
    getInput<float>("timeout", _param_timeout);
    _traj_complete = false;
    _time_diff = 0;
    _target_reached = false;
    _trajectory_done = true;
    _is_alive = true;
    _launch_time = std::chrono::system_clock::now();

    _controller_info_sub = _nh.subscribe("/proc_control/controller_info", 1, &WaitTargetReached::get_controller_info_cb, this);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitTargetReached::onRunning()
{
    if (!_is_alive)
    {
        return BT::NodeStatus::FAILURE;
    }

    if (_traj_complete)
    {
        std::chrono::duration<double> diff = std::chrono::system_clock::now() - _launch_time;
        _time_diff = diff.count();
    }

    if (_time_diff > _param_timeout || _target_reached)
    {
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

void WaitTargetReached::onHalted()
{
}

void WaitTargetReached::get_controller_info_cb(const sonia_common::MpcInfo &msg)
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

sonia_common::MissionTimer WaitTargetReached::missionTimerFunc(std::string mission, float timeout, time_t uniqueID, int status)
{
    sonia_common::MissionTimer buffer;
    buffer.mission = mission;
    buffer.timeout = timeout;
    buffer.uniqueID = std::to_string(uniqueID);
    buffer.status = status;
    return buffer;
}