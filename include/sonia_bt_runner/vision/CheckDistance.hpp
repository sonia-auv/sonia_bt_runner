#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"
#include "sonia_common_ros2/msg/detection.hpp"


namespace vision{
    class CheckDistance: public BT::StatefulActionNode{
        public:
            CheckDistance(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~CheckDistance();
            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<sonia_common_ros2::msg::Detection>("Object"),
                    BT::OutputPort<float>("TranslationX")};
            }

            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;
        private:
            std::shared_ptr<rclcpp::Node> ros_node;

            BT::Expected<sonia_common_ros2::msg::Detection> object_concerned;
            double translationX;
            
    };

}