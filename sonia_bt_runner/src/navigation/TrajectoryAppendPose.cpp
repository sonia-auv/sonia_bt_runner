#include "sonia_bt_runner/navigation/TrajectoryAppendPose.hpp"

namespace navigation{
    TrajectoryAppendPose::TrajectoryAppendPose(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
    {
    }

    BT::NodeStatus TrajectoryAppendPose::tick()
    {
        TRAJECTORY_APPEND_POSE_POSITION_X_TYPE positionX = 0.0;
        getInput<TRAJECTORY_APPEND_POSE_POSITION_X_TYPE>(TRAJECTORY_APPEND_POSE_POSITION_X, positionX);
        TRAJECTORY_APPEND_POSE_POSITION_Y_TYPE positionY = 0.0;
        getInput<TRAJECTORY_APPEND_POSE_POSITION_Y_TYPE>(TRAJECTORY_APPEND_POSE_POSITION_Y, positionY);
        TRAJECTORY_APPEND_POSE_POSITION_Z_TYPE positionZ = 0.0;
        getInput<TRAJECTORY_APPEND_POSE_POSITION_Z_TYPE>(TRAJECTORY_APPEND_POSE_POSITION_Z, positionZ);
        TRAJECTORY_APPEND_POSE_ORIENTATION_X_TYPE orientationX = 0.0;
        getInput<TRAJECTORY_APPEND_POSE_ORIENTATION_X_TYPE>(TRAJECTORY_APPEND_POSE_ORIENTATION_X, orientationX);
        TRAJECTORY_APPEND_POSE_ORIENTATION_Y_TYPE orientationY = 0.0;
        getInput<TRAJECTORY_APPEND_POSE_ORIENTATION_Y_TYPE>(TRAJECTORY_APPEND_POSE_ORIENTATION_Y, orientationY);
        TRAJECTORY_APPEND_POSE_ORIENTATION_Z_TYPE orientationZ = 0.0;
        getInput<TRAJECTORY_APPEND_POSE_ORIENTATION_Z_TYPE>(TRAJECTORY_APPEND_POSE_ORIENTATION_Z, orientationZ);
        TRAJECTORY_APPEND_POSE_FRAME_TYPE frame = 1;
        getInput<TRAJECTORY_APPEND_POSE_FRAME_TYPE>(TRAJECTORY_APPEND_POSE_FRAME, frame);
        TRAJECTORY_APPEND_POSE_SPEED_TYPE speed = 0;
        getInput<TRAJECTORY_APPEND_POSE_SPEED_TYPE>(TRAJECTORY_APPEND_POSE_SPEED, speed);
        TRAJECTORY_APPEND_POSE_PRECISION_TYPE precision = 0;
        getInput<TRAJECTORY_APPEND_POSE_PRECISION_TYPE>(TRAJECTORY_APPEND_POSE_PRECISION, precision);
        TRAJECTORY_APPEND_POSE_LONG_ROTATION_TYPE long_rotation = false;
        getInput<TRAJECTORY_APPEND_POSE_LONG_ROTATION_TYPE>(TRAJECTORY_APPEND_POSE_LONG_ROTATION, long_rotation);

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

        TRAJECTORY_APPEND_POSE_TRAJECTORY_TYPE t;
        getInput<TRAJECTORY_APPEND_POSE_TRAJECTORY_TYPE>(TRAJECTORY_APPEND_POSE_TRAJECTORY, t);
        t.trajectory.push_back(tp);
        setOutput(TRAJECTORY_APPEND_POSE_TRAJECTORY, t);

        return BT::NodeStatus::SUCCESS;
    }
}
