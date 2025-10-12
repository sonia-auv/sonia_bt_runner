#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_bt_runner/utils/MainAlignment.hpp"


namespace navigation
{

    class MoveToMission : public BT::SyncActionNode
    {
        public:
            MoveToMission(const std::string &name, const BT::NodeConfig &config);
            ~MoveToMission()=default;
            static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<AiDetectionArray>("Detections"),

                    // Outputs
                    BT::BidirectionalPort<Trajectory>("Trajectory"),
                };
            }

            BT::NodeStatus executeTick() override;


        private:
            std::shared_ptr<rclcpp::Node> ros_node;
            int valid;
            BT::Expected<AiDetectionArray> arr;

    };

}  // namespace navigation
