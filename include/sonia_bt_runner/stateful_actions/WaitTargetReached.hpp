#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "ros/ros.h"
#include "sonia_common/MissionTimer.h"
#include "sonia_common/MpcInfo.h"

class WaitTargetReached : public BT::StatefulActionNode
{
public:
    WaitTargetReached(const std::string &name, const BT::NodeConfig &config);

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

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:
    void get_controller_info_cb(const sonia_common::MpcInfo &msg);

    sonia_common::MissionTimer missionTimerFunc(std::string mission, float timeout, time_t uniqueID, int status);

    ros::NodeHandle _nh;
    ros::Publisher _timeout_pub;
    ros::Subscriber _controller_info_sub;

    std::chrono::_V2::system_clock::time_point _launch_time;

    bool _traj_complete;
    float _time_diff;
    bool _target_reached;
    bool _trajectory_done_prev;
    bool _trajectory_done;
    bool _is_alive;
    float _param_timeout;
};