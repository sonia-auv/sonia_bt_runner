#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"


#define FACE_THE_OBJECT_DETECTION_NAME "Detection"
#define FACE_THE_OBJECT_DETECTION_TYPE AiDetection
#define FACE_THE_OBJECT_TRAJECTORY_NAME "Trajectory"
#define FACE_THE_OBJECT_TRAJECTORY_TYPE Trajectory


namespace navigation
{

    class FaceTheObject : public BT::SyncActionNode
    {
        public:
            FaceTheObject(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~FaceTheObject() override = default;
            static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<FACE_THE_OBJECT_DETECTION_TYPE>(FACE_THE_OBJECT_DETECTION_NAME),

                    // Outputs
                    BT::BidirectionalPort<FACE_THE_OBJECT_TRAJECTORY_TYPE>(FACE_THE_OBJECT_TRAJECTORY_NAME),
                };
            }

            BT::NodeStatus tick() override;
        
        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
    };

}  // namespace navigation
