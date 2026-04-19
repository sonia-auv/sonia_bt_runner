#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_bt_runner/utils/AlignmentConstant.hpp"

namespace navigation
{

    class ObjectDrifting : public BT::SyncActionNode
    {
        public:
            ObjectDrifting(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~ObjectDrifting() override = default;
            static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<AiDetection>("Detection"),

                    // Outputs
                    BT::BidirectionalPort<Trajectory>("Trajectory"),
                };
            }
            BT::NodeStatus tick() override;
        
        private:
            std::shared_ptr<rclcpp::Node> ros_node;
            
    };

}  // namespace navigation
