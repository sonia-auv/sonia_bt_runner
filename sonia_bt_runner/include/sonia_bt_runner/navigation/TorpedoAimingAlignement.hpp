#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_bt_runner/utils/MainAlignment.hpp"

namespace navigation
{

    class TorpedoAimingAlignement : public BT::SyncActionNode
    {
        public:
            TorpedoAimingAlignement(const std::string &name, const BT::NodeConfig &config);
            ~TorpedoAimingAlignement()=default;
            static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<AiDetectionArray>("Detections"),
                    BT::InputPort<std::string>("LaunchingSide"),


                    // Outputs
                    BT::BidirectionalPort<Trajectory>("Trajectory"),
                };
            }
            BT::NodeStatus tick() override;
    };

}  // namespace navigation
