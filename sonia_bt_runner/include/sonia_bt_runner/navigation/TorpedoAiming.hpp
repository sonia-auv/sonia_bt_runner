#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_bt_runner/utils/CONSTANT.hpp"

#define TORPEDO_AIMING_DETECTION_NAME "Detection"
#define TORPEDO_AIMING_DETECTION_TYPE AiDetection
#define TORPEDO_AIMING_LAUNCHING_SIDE_NAME "LaunchingSide"
#define TORPEDO_AIMING_LAUNCHING_SIDE_TYPE std::string
#define TORPEDO_AIMING_TARGET_OFFSET_NAME "TargetOffset"
#define TORPEDO_AIMING_TARGET_OFFSET_TYPE float
#define TORPEDO_AIMING_TRAJECTORY_NAME "Trajectory"
#define TORPEDO_AIMING_TRAJECTORY_TYPE Trajectory

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
                    BT::InputPort<TORPEDO_AIMING_DETECTION_TYPE>(TORPEDO_AIMING_DETECTION_NAME),
                    BT::InputPort<TORPEDO_AIMING_LAUNCHING_SIDE_TYPE>(TORPEDO_AIMING_LAUNCHING_SIDE_NAME),
                    BT::InputPort<TORPEDO_AIMING_TARGET_OFFSET_TYPE>(TORPEDO_AIMING_TARGET_OFFSET_NAME),

                    // Outputs
                    BT::BidirectionalPort<TORPEDO_AIMING_TRAJECTORY_TYPE>(TORPEDO_AIMING_TRAJECTORY_NAME),
                };
            }
            BT::NodeStatus tick() override;
        
        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
    };

}  // namespace navigation
