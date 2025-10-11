#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
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
                    // Inputs
                    BT::InputPort<int>("Camera", 1, "1: Front, 0: Bottom"),
                    BT::InputPort<std::string>("Object_class", "Searched object"),
                    BT::InputPort<float>("Confidence", 0.6, "Ai confidence"),
                    BT::InputPort<int>("Min_size_output", "minumum number of frames with at least one detection before sending results"),
                    BT::InputPort<int>("Max_size_output", "Max size of the output array"),
                    BT::InputPort<int>("Max_frame_before_failling", "maximum allowed frames"),
                    BT::InputPort<float>("Max_depth", "Maximum allowed depth"),

                    // Outputs
                    BT::OutputPort<AiDetectionArray>("Detected_object_array")};
            }

            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;
        private:
            void ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg);
            std::shared_ptr<rclcpp::Node> ros_node;
            rclcpp::Subscription<sonia_common_ros2::msg::DetectionArray>::SharedPtr ai_filter_sub;

            std::vector<sonia_common_ros2::msg::Detection> _detection_array;
            int counter;
            int nb_detection;

            BT::Expected<std::string> _object;
            BT::Expected<int> max_frame_before_failling;
            BT::Expected<int> max_size_output;
            BT::Expected<float> confidence;
            BT::Expected<float> max_depth;
            BT::Expected<int> min_size_output;      
    };
}