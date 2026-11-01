#pragma once

#include <limits>

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/CONSTANT.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"

#define MOVE_ABOVE_OBJECT_DETECTION "Detection"
#define MOVE_ABOVE_OBJECT_DETECTION_TYPE AiDetection

#define MOVE_ABOVE_OBJECT_TRAJECTORY "Trajectory"
#define MOVE_ABOVE_OBJECT_TRAJECTORY_TYPE Trajectory

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
                BT::InputPort<MOVE_ABOVE_OBJECT_DETECTION_TYPE>(MOVE_ABOVE_OBJECT_DETECTION),

                // Outputs
                BT::BidirectionalPort<MOVE_ABOVE_OBJECT_TRAJECTORY_TYPE>(MOVE_ABOVE_OBJECT_TRAJECTORY),
            };
        }

        BT::NodeStatus tick() override;

        private:
        std::shared_ptr<rclcpp::Node> _ros_node;
    };

}  // namespace navigation
