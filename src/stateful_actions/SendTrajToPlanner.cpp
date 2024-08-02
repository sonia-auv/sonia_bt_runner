#include "sonia_bt_runner/stateful_actions/SendTrajToPlanner.hpp"

SendTrajToPlanner::SendTrajToPlanner(const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config), _valid(0), _time_launch(std::chrono::system_clock::now())
{

    _planner_pub = _nh.advertise<sonia_common::MultiAddPose>("/proc_planner/send_multi_addpose", 1, true);
}

BT::NodeStatus SendTrajToPlanner::onStart()
{
    ROS_INFO("Starting SendTrajToPlanner");
    Trajectory t;
    getInput<Trajectory>("trajectory", t);
    sonia_common::MultiAddPose array_to_send;
    int interpolation = 0;
    getInput<int>("interpolation", interpolation);

    array_to_send.interpolation_method = interpolation;
    for (size_t i = 0; i < t.trajectory.size(); i++)
    {
        sonia_common::AddPose ap;
        ap.position.x = t.trajectory[i].positionX;
        ap.position.y = t.trajectory[i].positionY;
        ap.position.z = t.trajectory[i].positionZ;
        ap.orientation.x = t.trajectory[i].orientationX;
        ap.orientation.y = t.trajectory[i].orientationY;
        ap.orientation.z = t.trajectory[i].orientationZ;
        ap.frame = t.trajectory[i].frame;
        ap.speed = t.trajectory[i].speed;
        ap.fine = t.trajectory[i].precision;
        ap.rotation = t.trajectory[i].long_rotation;
        array_to_send.pose.push_back(ap);
    }
    _planner_pub.publish(array_to_send);
    _time_launch = std::chrono::system_clock::now();
    _trajectory_compiled = _nh.subscribe("/proc_planner/is_waypoints_valid", 1, &SendTrajToPlanner::is_waypoints_valid_cb, this);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SendTrajToPlanner::onRunning()
{
    std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - _time_launch;
    if (elapsed_time.count() > 5)
    {
        if (_valid == 0)
        {
            ROS_INFO("Waypoints Valid");
            _trajectory_compiled.shutdown();

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_INFO("Waypoints Invalid");
            _trajectory_compiled.shutdown();

            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::RUNNING;
}

void SendTrajToPlanner::onHalted()
{
}

void SendTrajToPlanner::is_waypoints_valid_cb(const std_msgs::Int8 &msg)
{
    _valid = msg.data;
}
