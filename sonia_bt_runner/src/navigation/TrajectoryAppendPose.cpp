#include "sonia_bt_runner/navigation/TrajectoryAppendPose.hpp"

namespace navigation{
    TrajectoryAppendPose::TrajectoryAppendPose(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
    {
    }

    BT::NodeStatus TrajectoryAppendPose::tick()
    {
        auto positionX = getInput<TRAJECTORY_APPEND_POSE_POSITION_X_TYPE>(TRAJECTORY_APPEND_POSE_POSITION_X_NAME).value();
        auto positionY = getInput<TRAJECTORY_APPEND_POSE_POSITION_Y_TYPE>(TRAJECTORY_APPEND_POSE_POSITION_Y_NAME).value();
        auto positionZ = getInput<TRAJECTORY_APPEND_POSE_POSITION_Z_TYPE>(TRAJECTORY_APPEND_POSE_POSITION_Z_NAME).value();
        auto orientationX = getInput<TRAJECTORY_APPEND_POSE_ORIENTATION_X_TYPE>(TRAJECTORY_APPEND_POSE_ORIENTATION_X_NAME).value();
        auto orientationY = getInput<TRAJECTORY_APPEND_POSE_ORIENTATION_Y_TYPE>(TRAJECTORY_APPEND_POSE_ORIENTATION_Y_NAME).value();
        auto orientationZ = getInput<TRAJECTORY_APPEND_POSE_ORIENTATION_Z_TYPE>(TRAJECTORY_APPEND_POSE_ORIENTATION_Z_NAME).value();
        auto frame = getInput<TRAJECTORY_APPEND_POSE_FRAME_TYPE>(TRAJECTORY_APPEND_POSE_FRAME_NAME).value();
        auto speed = getInput<TRAJECTORY_APPEND_POSE_SPEED_TYPE>(TRAJECTORY_APPEND_POSE_SPEED_NAME).value();
        auto precision = getInput<TRAJECTORY_APPEND_POSE_PRECISION_TYPE>(TRAJECTORY_APPEND_POSE_PRECISION_NAME).value();
        auto long_rotation = getInput<TRAJECTORY_APPEND_POSE_LONG_ROTATION_TYPE>(TRAJECTORY_APPEND_POSE_LONG_ROTATION_NAME).value();

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

        auto t = getInput<TRAJECTORY_APPEND_POSE_TRAJECTORY_TYPE>(TRAJECTORY_APPEND_POSE_TRAJECTORY_NAME).value();
        t.trajectory.push_back(tp);
        setOutput(TRAJECTORY_APPEND_POSE_TRAJECTORY_NAME, t);

        return BT::NodeStatus::SUCCESS;
    }
}