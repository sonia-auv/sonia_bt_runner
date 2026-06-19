#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_bt_runner/utils/CONSTANT.hpp"


#define OBJECT_DRIFTING_DETECTION_PARAM_NAME "Detection"
#define OBJECT_DRIFTING_DETECTION_PARAM_TYPE AiDetection
#define OBJECT_DRIFTING_TRAJECTORY_PARAM_NAME "Trajectory"
#define OBJECT_DRIFTING_TRAJECTORY_PARAM_TYPE Trajectory

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
                    BT::InputPort<OBJECT_DRIFTING_DETECTION_PARAM_TYPE>(OBJECT_DRIFTING_DETECTION_PARAM_NAME),

                    // Outputs
                    BT::BidirectionalPort<OBJECT_DRIFTING_TRAJECTORY_PARAM_TYPE>(OBJECT_DRIFTING_TRAJECTORY_PARAM_NAME),
                };
            }
            BT::NodeStatus tick() override;
        
        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
            
    };

}  // namespace navigation
