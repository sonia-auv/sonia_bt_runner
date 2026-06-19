#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"


#define FACE_THE_OBJECT_DETECTION_PARAM_NAME "Detection"
#define FACE_THE_OBJECT_DETECTION_PARAM_TYPE AiDetection
#define FACE_THE_OBJECT_TRAJECTORY_PARAM_NAME "Trajectory"
#define FACE_THE_OBJECT_TRAJECTORY_PARAM_TYPE Trajectory


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
                    BT::InputPort<FACE_THE_OBJECT_DETECTION_PARAM_TYPE>(FACE_THE_OBJECT_DETECTION_PARAM_NAME),

                    // Outputs
                    BT::BidirectionalPort<FACE_THE_OBJECT_TRAJECTORY_PARAM_TYPE>(FACE_THE_OBJECT_TRAJECTORY_PARAM_NAME),
                };
            }

            BT::NodeStatus tick() override;
        
        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
    };

}  // namespace navigation
