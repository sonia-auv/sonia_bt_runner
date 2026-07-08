#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_bt_runner/utils/Point.hpp"

#define MOVE_TO_POINT_POINT "Point"
#define MOVE_TO_POINT_POINT_TYPE Point

#define MOVE_TO_POINT_TRAJECTORY "Trajectory"
#define MOVE_TO_POINT_TRAJECTORY_TYPE Trajectory

namespace navigation {

    class MoveToPoint : public BT::SyncActionNode {

        public:
            MoveToPoint(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~MoveToPoint() override = default;
            
            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<MOVE_TO_POINT_POINT_TYPE>(MOVE_TO_POINT_POINT),
                    BT::BidirectionalPort<MOVE_TO_POINT_TRAJECTORY_TYPE>(MOVE_TO_POINT_TRAJECTORY),
                };
            }

            BT::NodeStatus tick() override;

        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
    };
}
