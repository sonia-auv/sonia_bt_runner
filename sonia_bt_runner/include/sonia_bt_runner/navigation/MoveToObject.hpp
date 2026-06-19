#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"


#define MOVE_TO_OBJECT_DETECTION_NAME "Detection"
#define MOVE_TO_OBJECT_DETECTION_TYPE AiDetection
#define MOVE_TO_OBJECT_OFFSET_NAME "Offset"
#define MOVE_TO_OBJECT_OFFSET_NAME float
#define MOVE_TO_OBJECT_TRAJECTORY_NAME "Trajectory"
#define MOVE_TO_OBJECT_TRAJECTORY_TYPE Trajectory

namespace navigation
{

    class MoveToObject : public BT::SyncActionNode
    {
        public:
        MoveToObject(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~MoveToObject() override = default;
            static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<MOVE_TO_OBJECT_DETECTION_TYPE>(MOVE_TO_OBJECT_DETECTION_NAME),
                    BT::InputPort<MOVE_TO_OBJECT_OFFSET_NAME>(MOVE_TO_OBJECT_OFFSET_NAME),

                    // Outputs
                    BT::BidirectionalPort<MOVE_TO_OBJECT_TRAJECTORY_TYPE>(MOVE_TO_OBJECT_TRAJECTORY_NAME),
                };
            }

            BT::NodeStatus tick() override;
        
        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
    };

}  // namespace navigation
