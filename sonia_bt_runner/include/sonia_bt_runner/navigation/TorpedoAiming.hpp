#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_bt_runner/utils/AlignmentConstant.hpp"

namespace navigation
{

    class TorpedoAiming : public BT::SyncActionNode
    {
        public:
            TorpedoAiming(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~TorpedoAiming() override = default;
            static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<AiDetection>("Detection"),
                    BT::InputPort<std::string>("LaunchingSide"),
                    BT::InputPort<float>("TargetOffset"),

                    // Outputs
                    BT::BidirectionalPort<Trajectory>("Trajectory"),
                };
            }
            BT::NodeStatus tick() override;
        
        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
    };

}  // namespace navigation
