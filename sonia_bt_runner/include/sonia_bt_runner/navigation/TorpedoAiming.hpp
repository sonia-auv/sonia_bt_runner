#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_bt_runner/utils/CONSTANT.hpp"

#define TORPEDO_AIMING_DETECTION_PARAM_NAME "Detection"
#define TORPEDO_AIMING_DETECTION_PARAM_TYPE AiDetection
#define TORPEDO_AIMING_LAUNCHING_SIDE_PARAM_NAME "LaunchingSide"
#define TORPEDO_AIMING_LAUNCHING_SIDE_PARAM_TYPE std::string
#define TORPEDO_AIMING_TARGET_OFFSET_PARAM_NAME "TargetOffset"
#define TORPEDO_AIMING_TARGET_OFFSET_PARAM_TYPE float
#define TORPEDO_AIMING_TRAJECTORY_PARAM_NAME "Trajectory"
#define TORPEDO_AIMING_TRAJECTORY_PARAM_TYPE Trajectory

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
                    BT::InputPort<TORPEDO_AIMING_DETECTION_PARAM_TYPE>(TORPEDO_AIMING_DETECTION_PARAM_NAME),
                    BT::InputPort<TORPEDO_AIMING_LAUNCHING_SIDE_PARAM_TYPE>(TORPEDO_AIMING_LAUNCHING_SIDE_PARAM_NAME),
                    BT::InputPort<TORPEDO_AIMING_TARGET_OFFSET_PARAM_TYPE>(TORPEDO_AIMING_TARGET_OFFSET_PARAM_NAME),

                    // Outputs
                    BT::BidirectionalPort<TORPEDO_AIMING_TRAJECTORY_PARAM_TYPE>(TORPEDO_AIMING_TRAJECTORY_PARAM_NAME),
                };
            }
            BT::NodeStatus tick() override;
        
        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
    };

}  // namespace navigation
