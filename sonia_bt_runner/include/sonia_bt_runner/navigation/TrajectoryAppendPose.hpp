#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/Trajectory.hpp"


#define TRAJECTORY_APPEND_POSE_POSITION_X_NAME "positionX"
#define TRAJECTORY_APPEND_POSE_POSITION_X_TYPE float
#define TRAJECTORY_APPEND_POSE_POSITION_Y_NAME "positionY"
#define TRAJECTORY_APPEND_POSE_POSITION_Y_TYPE float
#define TRAJECTORY_APPEND_POSE_POSITION_Z_NAME "positionZ"
#define TRAJECTORY_APPEND_POSE_POSITION_Z_TYPE float
#define TRAJECTORY_APPEND_POSE_ORIENTATION_X_NAME "orientationX"
#define TRAJECTORY_APPEND_POSE_ORIENTATION_X_TYPE float
#define TRAJECTORY_APPEND_POSE_ORIENTATION_Y_NAME "orientationY"
#define TRAJECTORY_APPEND_POSE_ORIENTATION_Y_TYPE float
#define TRAJECTORY_APPEND_POSE_ORIENTATION_Z_NAME "orientationZ"
#define TRAJECTORY_APPEND_POSE_ORIENTATION_Z_TYPE float
#define TRAJECTORY_APPEND_POSE_FRAME_NAME "frame"
#define TRAJECTORY_APPEND_POSE_FRAME_TYPE int
#define TRAJECTORY_APPEND_POSE_SPEED_NAME "speed"
#define TRAJECTORY_APPEND_POSE_SPEED_TYPE int
#define TRAJECTORY_APPEND_POSE_PRECISION_NAME "precision"
#define TRAJECTORY_APPEND_POSE_PRECISION_TYPE int
#define TRAJECTORY_APPEND_POSE_LONG_ROTATION_NAME "longRotation"
#define TRAJECTORY_APPEND_POSE_LONG_ROTATION_TYPE bool
#define TRAJECTORY_APPEND_POSE_TRAJECTORY_NAME "trajectory"
#define TRAJECTORY_APPEND_POSE_TRAJECTORY_TYPE Trajectory

namespace navigation{
    class TrajectoryAppendPose : public BT::SyncActionNode
    {
        public:
            TrajectoryAppendPose(const std::string &name, const BT::NodeConfig &config);
            ~TrajectoryAppendPose() override = default;

            static BT::PortsList providedPorts()
            {
                const TRAJECTORY_APPEND_POSE_POSITION_X_TYPE def_positionX = 0.0;
                const TRAJECTORY_APPEND_POSE_POSITION_Y_TYPE def_positionY = 0.0;
                const TRAJECTORY_APPEND_POSE_POSITION_Z_TYPE def_positionZ = 0.0;
                const TRAJECTORY_APPEND_POSE_ORIENTATION_X_TYPE def_orientationX = 0.0;
                const TRAJECTORY_APPEND_POSE_ORIENTATION_Y_TYPE def_orientationY = 0.0;
                const TRAJECTORY_APPEND_POSE_ORIENTATION_Z_TYPE def_orientationZ = 0.0;
                const TRAJECTORY_APPEND_POSE_FRAME_TYPE def_frame = 1;
                const TRAJECTORY_APPEND_POSE_SPEED_TYPE def_speed = 0;
                const TRAJECTORY_APPEND_POSE_PRECISION_TYPE def_precision = 0;
                const TRAJECTORY_APPEND_POSE_LONG_ROTATION_TYPE def_longRotation = false;
                return {
                    BT::InputPort<TRAJECTORY_APPEND_POSE_POSITION_X_TYPE>(TRAJECTORY_APPEND_POSE_POSITION_X_NAME, def_positionX, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_POSITION_Y_TYPE>(TRAJECTORY_APPEND_POSE_POSITION_Y_NAME, def_positionY, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_POSITION_Z_TYPE>(TRAJECTORY_APPEND_POSE_POSITION_Z_NAME, def_positionZ, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_ORIENTATION_X_TYPE>(TRAJECTORY_APPEND_POSE_ORIENTATION_X_NAME, def_orientationX, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_ORIENTATION_Y_TYPE>(TRAJECTORY_APPEND_POSE_ORIENTATION_Y_NAME, def_orientationY, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_ORIENTATION_Z_TYPE>(TRAJECTORY_APPEND_POSE_ORIENTATION_Z_NAME, def_orientationZ, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_FRAME_TYPE>(TRAJECTORY_APPEND_POSE_FRAME_NAME, def_frame, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_SPEED_TYPE>(TRAJECTORY_APPEND_POSE_SPEED_NAME, def_speed, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_PRECISION_TYPE>(TRAJECTORY_APPEND_POSE_PRECISION_NAME, def_precision, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_LONG_ROTATION_TYPE>(TRAJECTORY_APPEND_POSE_LONG_ROTATION_NAME, def_longRotation, ""),
                    BT::BidirectionalPort<TRAJECTORY_APPEND_POSE_TRAJECTORY_TYPE>(TRAJECTORY_APPEND_POSE_TRAJECTORY_NAME),
                };
            }

            BT::NodeStatus tick() override;
    };
}