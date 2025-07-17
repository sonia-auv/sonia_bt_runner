#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"
#include "sonia_common_ros2/msg/detection.hpp"
#include "sonia_common_ros2/msg/detection_array.hpp"


namespace vision{
    class AiFilter: public BT::StatefulActionNode{
        public:
            AiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~AiFilter();
            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<int>("Object", "Searched object"),
                    BT::InputPort<float>("Confidence", 0.6, "Ai confidence"),
                    BT::InputPort<int>("Expected_count", "object count"),
                    BT::InputPort<int>("Max_frame", "maximum allowed frames"),
                    BT::InputPort<int>("Max_depth", "Maximum allowed depth")};
                    BT::OutputPort<AiDetectionArray>("ai_info");
            }

            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;
        private:
            void ai_front_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg);
            void ai_bottom_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg);

            std::shared_ptr<rclcpp::Node> ros_node;
            rclcpp::Subscription<sonia_common_ros2::msg::DetectionArray>::SharedPtr ai_front_filter_sub;
            rclcpp::Subscription<sonia_common_ros2::msg::DetectionArray>::SharedPtr ai_bottom_filter_sub;

            
    };

}