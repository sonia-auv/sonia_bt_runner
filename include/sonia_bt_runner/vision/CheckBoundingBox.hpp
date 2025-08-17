#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"
#include "sonia_common_ros2/msg/detection.hpp"
#include "sonia_common_ros2/msg/detection_array.hpp"


namespace vision{
    class CheckBoundingBox: public BT::StatefulActionNode{
        public:
            CheckBoundingBox(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~CheckBoundingBox();
            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<int>("Previous_width"),
                    BT::InputPort<int>("Width_growing"),
                    BT::OutputPort<int>("New_width_growth"),
                    BT::OutputPort<int>("New_width"),
                    BT::InputPort<AiDetectionArray>("Object")
                };

            }

            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;
        private:
            std::shared_ptr<rclcpp::Node> ros_node;

            BT::Expected<AiDetectionArray> object_concerned;
            AiDetection object_concerned2;
            BT::Expected<int> previous_width;
            BT::Expected<int> current_width;
            BT::Expected<int> growing_width;
            
    };

}