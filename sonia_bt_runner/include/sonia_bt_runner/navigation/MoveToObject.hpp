#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"


namespace navigation
{

    class MoveToObject : public BT::SyncActionNode
    {
        public:
        MoveToObject(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~MoveToObject() override = default;
            static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<AiDetection>("Detection"),
                    BT::InputPort<float>("Offset"),

                    // Outputs
                    BT::BidirectionalPort<Trajectory>("Trajectory"),
                };
            }

            BT::NodeStatus tick() override;
        
        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
    };

}  // namespace navigation
