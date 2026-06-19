#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/Trajectory.hpp"


#define TRAJECTORY_APPEND_POSE_POSITION_X_PARAM_NAME "positionX"
#define TRAJECTORY_APPEND_POSE_POSITION_X_PARAM_TYPE float
#define TRAJECTORY_APPEND_POSE_POSITION_Y_PARAM_NAME "positionY"
#define TRAJECTORY_APPEND_POSE_POSITION_Y_PARAM_TYPE float
#define TRAJECTORY_APPEND_POSE_POSITION_Z_PARAM_NAME "positionZ"
#define TRAJECTORY_APPEND_POSE_POSITION_Z_PARAM_TYPE float
#define TRAJECTORY_APPEND_POSE_ORIENTATION_X_PARAM_NAME "orientationX"
#define TRAJECTORY_APPEND_POSE_ORIENTATION_X_PARAM_TYPE float
#define TRAJECTORY_APPEND_POSE_ORIENTATION_Y_PARAM_NAME "orientationY"
#define TRAJECTORY_APPEND_POSE_ORIENTATION_Y_PARAM_TYPE float
#define TRAJECTORY_APPEND_POSE_ORIENTATION_Z_PARAM_NAME "orientationZ"
#define TRAJECTORY_APPEND_POSE_ORIENTATION_Z_PARAM_TYPE float
#define TRAJECTORY_APPEND_POSE_FRAME_PARAM_NAME "frame"
#define TRAJECTORY_APPEND_POSE_FRAME_PARAM_TYPE int
#define TRAJECTORY_APPEND_POSE_SPEED_PARAM_NAME "speed"
#define TRAJECTORY_APPEND_POSE_SPEED_PARAM_TYPE int
#define TRAJECTORY_APPEND_POSE_PRECISION_PARAM_NAME "precision"
#define TRAJECTORY_APPEND_POSE_PRECISION_PARAM_TYPE int
#define TRAJECTORY_APPEND_POSE_LONG_ROTATION_PARAM_NAME "longRotation"
#define TRAJECTORY_APPEND_POSE_LONG_ROTATION_PARAM_TYPE bool
#define TRAJECTORY_APPEND_POSE_TRAJECTORY_PARAM_NAME "trajectory"
#define TRAJECTORY_APPEND_POSE_TRAJECTORY_PARAM_TYPE Trajectory

namespace navigation{
    class TrajectoryAppendPose : public BT::SyncActionNode
    {
        public:
            TrajectoryAppendPose(const std::string &name, const BT::NodeConfig &config);
            ~TrajectoryAppendPose() override = default;

            static BT::PortsList providedPorts()
            {
                const TRAJECTORY_APPEND_POSE_POSITION_X_PARAM_TYPE def_positionX = 0.0;
                const TRAJECTORY_APPEND_POSE_POSITION_Y_PARAM_TYPE def_positionY = 0.0;
                const TRAJECTORY_APPEND_POSE_POSITION_Z_PARAM_TYPE def_positionZ = 0.0;
                const TRAJECTORY_APPEND_POSE_ORIENTATION_X_PARAM_TYPE def_orientationX = 0.0;
                const TRAJECTORY_APPEND_POSE_ORIENTATION_Y_PARAM_TYPE def_orientationY = 0.0;
                const TRAJECTORY_APPEND_POSE_ORIENTATION_Z_PARAM_TYPE def_orientationZ = 0.0;
                const TRAJECTORY_APPEND_POSE_FRAME_PARAM_TYPE def_frame = 1;
                const TRAJECTORY_APPEND_POSE_SPEED_PARAM_TYPE def_speed = 0;
                const TRAJECTORY_APPEND_POSE_PRECISION_PARAM_TYPE def_precision = 0;
                const TRAJECTORY_APPEND_POSE_LONG_ROTATION_PARAM_TYPE def_longRotation = false;
                return {
                    BT::InputPort<TRAJECTORY_APPEND_POSE_POSITION_X_PARAM_TYPE>(TRAJECTORY_APPEND_POSE_POSITION_X_PARAM_NAME, def_positionX, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_POSITION_Y_PARAM_TYPE>(TRAJECTORY_APPEND_POSE_POSITION_Y_PARAM_NAME, def_positionY, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_POSITION_Z_PARAM_TYPE>(TRAJECTORY_APPEND_POSE_POSITION_Z_PARAM_NAME, def_positionZ, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_ORIENTATION_X_PARAM_TYPE>(TRAJECTORY_APPEND_POSE_ORIENTATION_X_PARAM_NAME, def_orientationX, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_ORIENTATION_Y_PARAM_TYPE>(TRAJECTORY_APPEND_POSE_ORIENTATION_Y_PARAM_NAME, def_orientationY, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_ORIENTATION_Z_PARAM_TYPE>(TRAJECTORY_APPEND_POSE_ORIENTATION_Z_PARAM_NAME, def_orientationZ, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_FRAME_PARAM_TYPE>(TRAJECTORY_APPEND_POSE_FRAME_PARAM_NAME, def_frame, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_SPEED_PARAM_TYPE>(TRAJECTORY_APPEND_POSE_SPEED_PARAM_NAME, def_speed, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_PRECISION_PARAM_TYPE>(TRAJECTORY_APPEND_POSE_PRECISION_PARAM_NAME, def_precision, ""),
                    BT::InputPort<TRAJECTORY_APPEND_POSE_LONG_ROTATION_PARAM_TYPE>(TRAJECTORY_APPEND_POSE_LONG_ROTATION_PARAM_NAME, def_longRotation, ""),
                    BT::BidirectionalPort<TRAJECTORY_APPEND_POSE_TRAJECTORY_PARAM_TYPE>(TRAJECTORY_APPEND_POSE_TRAJECTORY_PARAM_NAME),
                };
            }

            BT::NodeStatus tick() override;
    };
}