#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_bt_runner/utils/Point.hpp"

#define MOVE_TO_POINT_POINT "Point"
#define MOVE_TO_POINT_POINT_TYPE Point

#define MOVE_TO_POINT_TRAJECTORY "Trajectory"
#define MOVE_TO_POINT_TRAJECTORY_TYPE Trajectory

#define MOVE_TO_POINT_OFFSET_POSITION_X "OffsetPositionX"
#define MOVE_TO_POINT_OFFSET_POSITION_X_PARAMS MOVE_TO_POINT_OFFSET_POSITION_X, 0.0, ""
#define MOVE_TO_POINT_OFFSET_POSITION_X_TYPE float

#define MOVE_TO_POINT_OFFSET_POSITION_Y "OffsetPositionY"
#define MOVE_TO_POINT_OFFSET_POSITION_Y_PARAMS MOVE_TO_POINT_OFFSET_POSITION_Y, 0.0, ""
#define MOVE_TO_POINT_OFFSET_POSITION_Y_TYPE float

#define MOVE_TO_POINT_OFFSET_POSITION_Z "OffsetPositionZ"
#define MOVE_TO_POINT_OFFSET_POSITION_Z_PARAMS MOVE_TO_POINT_OFFSET_POSITION_Z, 0.0, ""
#define MOVE_TO_POINT_OFFSET_POSITION_Z_TYPE float

namespace navigation {

    class MoveToPoint : public BT::SyncActionNode {

        public:
            MoveToPoint(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~MoveToPoint() override = default;
            
            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<MOVE_TO_POINT_POINT_TYPE>(MOVE_TO_POINT_POINT),
                    BT::InputPort<MOVE_TO_POINT_OFFSET_POSITION_X_TYPE>(MOVE_TO_POINT_OFFSET_POSITION_X_PARAMS),
                    BT::InputPort<MOVE_TO_POINT_OFFSET_POSITION_Y_TYPE>(MOVE_TO_POINT_OFFSET_POSITION_Y_PARAMS),
                    BT::InputPort<MOVE_TO_POINT_OFFSET_POSITION_Z_TYPE>(MOVE_TO_POINT_OFFSET_POSITION_Z_PARAMS),
                    BT::BidirectionalPort<MOVE_TO_POINT_TRAJECTORY_TYPE>(MOVE_TO_POINT_TRAJECTORY),
                };
            }

            BT::NodeStatus tick() override;

        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
    };
}
