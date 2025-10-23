#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_bt_runner/utils/AlignmentConstant.hpp"


namespace navigation
{

    class MoveToObject : public BT::SyncActionNode
    {
        public:
        MoveToObject(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~MoveToObject()=default;
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
