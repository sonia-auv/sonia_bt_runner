#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_bt_runner/utils/MainAlignment.hpp"

namespace navigation
{

    class TorpedoDriftingAlignement : public BT::AsyncActionNode
    {
        public:
            TorpedoDriftingAlignement(const std::string &name, const BT::NodeConfig &config,std::shared_ptr<rclcpp::Node> node);
            ~TorpedoDriftingAlignement()=default;
            static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<AiDetectionArray>("Detections"),

                    // Outputs
                    BT::BidirectionalPort<Trajectory>("Trajectory"),
                };
            }
            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;

        private:
            std::shared_ptr<rclcpp::Node> ros_node;
            int valid;
            BT::Expected<AiDetectionArray> arr;

    };

}  // namespace navigation
