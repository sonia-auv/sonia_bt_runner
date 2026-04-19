#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/Trajectory.hpp"

namespace navigation{
    class TrajectoryAppendPose : public BT::SyncActionNode
    {
        public:
            TrajectoryAppendPose(const std::string &name, const BT::NodeConfig &config);
            ~TrajectoryAppendPose() override = default;

            static BT::PortsList providedPorts()
            {
                const float def_positionX = 0.0;
                const float def_positionY = 0.0;
                const float def_positionZ = 0.0;
                const float def_orientationX = 0.0;
                const float def_orientationY = 0.0;
                const float def_orientationZ = 0.0;
                const int def_frame = 1;
                const int def_speed = 0;
                const int def_precision = 0;
                const bool def_longRotation = false;
                return {
                    BT::InputPort<float>("positionX", def_positionX, ""),
                    BT::InputPort<float>("positionY", def_positionY, ""),
                    BT::InputPort<float>("positionZ", def_positionZ, ""),
                    BT::InputPort<float>("orientationX", def_orientationX, ""),
                    BT::InputPort<float>("orientationY", def_orientationY, ""),
                    BT::InputPort<float>("orientationZ", def_orientationZ, ""),
                    BT::InputPort<int>("frame", def_frame, ""),
                    BT::InputPort<int>("speed", def_speed, ""),
                    BT::InputPort<int>("precision", def_precision, ""),
                    BT::InputPort<bool>("longRotation", def_longRotation, ""),
                    BT::BidirectionalPort<Trajectory>("trajectory"),
                };
            }

            BT::NodeStatus tick() override;
    };
}