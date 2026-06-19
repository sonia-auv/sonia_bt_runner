#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_bt_runner/utils/CONSTANT.hpp"


#define MOVE_ABOVE_OBJECT_DETECTION_PARAM_NAME "Detection"
#define MOVE_ABOVE_OBJECT_DETECTION_PARAM_TYPE AiDetection
#define MOVE_ABOVE_OBJECT_TRAJECTORY_PARAM_NAME "Trajectory"
#define MOVE_ABOVE_OBJECT_TRAJECTORY_PARAM_TYPE Trajectory

namespace navigation
{

    class MoveAboveObject : public BT::SyncActionNode
    {
        public:
            MoveAboveObject(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~MoveAboveObject() override = default;
            static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<MOVE_ABOVE_OBJECT_DETECTION_PARAM_TYPE>(MOVE_ABOVE_OBJECT_DETECTION_PARAM_NAME),

                    // Outputs
                    BT::BidirectionalPort<MOVE_ABOVE_OBJECT_TRAJECTORY_PARAM_TYPE>(MOVE_ABOVE_OBJECT_TRAJECTORY_PARAM_NAME),
                };
            }

            BT::NodeStatus tick() override;
            
        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
    };

}  // namespace navigation
