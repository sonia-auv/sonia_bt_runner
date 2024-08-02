#include "sonia_bt_runner/sync_actions/AddPoseToTraj.hpp"

BT::NodeStatus AddPoseToTraj::tick()
{
    float positionX = 0.0;
    getInput<float>("positionX", positionX);
    float positionY = 0.0;
    getInput<float>("positionY", positionY);
    float positionZ = 0.0;
    getInput<float>("positionZ", positionZ);
    float orientationX = 0.0;
    getInput<float>("orientationX", orientationX);
    float orientationY = 0.0;
    getInput<float>("orientationY", orientationY);
    float orientationZ = 0.0;
    getInput<float>("orientationZ", orientationZ);
    int frame = 1;
    getInput<int>("frame", frame);
    int speed = 0;
    getInput<int>("speed", speed);
    int precision = 0;
    getInput<int>("precision", precision);
    bool long_rotation = false;
    getInput<bool>("longRotation", long_rotation);

    TrajectoryPose tp;
    tp.positionX = positionX;
    tp.positionY = positionY;
    tp.positionZ = positionZ;
    tp.orientationX = orientationX;
    tp.orientationY = orientationY;
    tp.orientationZ = orientationZ;
    tp.frame = frame;
    tp.speed = speed;
    tp.precision = precision;
    tp.long_rotation = long_rotation;

    Trajectory t;
    getInput<Trajectory>("trajectory", t);
    t.trajectory.push_back(tp);
    setOutput("trajectory", t);

    return BT::NodeStatus::SUCCESS;
}