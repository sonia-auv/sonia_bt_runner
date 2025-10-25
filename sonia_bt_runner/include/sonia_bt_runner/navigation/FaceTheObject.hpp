#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"


namespace navigation
{

    class FaceTheObject : public BT::SyncActionNode
    {
        public:
            FaceTheObject(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~FaceTheObject()=default;
            static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<AiDetectionArray>("Detections"),

                    // Outputs
                    BT::BidirectionalPort<Trajectory>("Trajectory"),
                };
            }

            BT::NodeStatus tick() override;
        
        private:
            std::shared_ptr<rclcpp::Node> ros_node;
    };

}  // namespace navigation
