#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"


namespace navigation
{

    class GetBetweenDetections : public BT::SyncActionNode
    {
        public:
        GetBetweenDetections(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
        ~GetBetweenDetections() override = default;

	static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<AiDetection>("DetectionA"),
                    BT::InputPort<AiDetection>("DetectionB"),

                    // Outputs
                    BT::BidirectionalPort<float>("PositionY"),
                };
            }

            BT::NodeStatus tick() override;

private:
            std::shared_ptr<rclcpp::Node> _ros_node;
    };
}
