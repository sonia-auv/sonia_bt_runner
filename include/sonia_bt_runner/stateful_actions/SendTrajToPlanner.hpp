#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "ros/ros.h"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_common/AddPose.h"
#include "sonia_common/MultiAddPose.h"
#include "std_msgs/Int8.h"

using namespace std::chrono_literals;

class SendTrajToPlanner : public BT::StatefulActionNode
{
public:
    SendTrajToPlanner(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config), _valid(0), _time_launch(std::chrono::system_clock::now())
    {

        _planner_pub = _nh.advertise<sonia_common::MultiAddPose>("/proc_planner/send_multi_addpose", 1, true);
    }

    static BT::PortsList providedPorts()
    {
        const int def_inter = 0;
        return {
            BT::InputPort<int>("interpolation", def_inter, ""),
            BT::InputPort<Trajectory>("trajectory"),
        };
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:
    void is_waypoints_valid_cb(const std_msgs::Int8 &msg);

    ros::NodeHandle _nh;
    ros::Publisher _planner_pub;
    ros::Subscriber _trajectory_compiled;

    std::chrono::_V2::system_clock::time_point _time_launch;
    int _valid;
};