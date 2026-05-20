#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_bt_runner/utils/Point.hpp"

namespace navigation {

    class MoveToPoint : public BT::SyncActionNode {

        public:
            MoveToPoint(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~MoveToPoint() override = default;
            
            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<Point>("Point"),
                    BT::BidirectionalPort<Trajectory>("Trajectory"),
                };
            }

            BT::NodeStatus tick() override;

        private:
            std::shared_ptr<rclcpp::Node> ros_node;
    };
}