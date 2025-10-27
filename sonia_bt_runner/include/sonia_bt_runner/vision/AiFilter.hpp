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
                    BT::InputPort<int>("Min_detections_before_success", 2, "Minumum number of frames with at least one detection before sending results. Need to be heigher than 2"),
                    BT::InputPort<int>("Two_objects_possible", 0, "0: Only one object can be detected, 1: Two or more object of the same class can be detected"),
                    BT::InputPort<int>("Max_frame_before_failing", 0, "Maximum allowed frames before failing the object research"),
                    BT::InputPort<float>("Max_time_before_failing_ms", 0.0, "Maximum allowed time in ms before failing the object research"),
                    BT::InputPort<float>("Max_depth", 25.0, "Maximum allowed depth"),

                    // Outputs
                    BT::OutputPort<AiDetection>("Detected_object")
                };
            }

            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;
        private:
            void ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg);
            std::shared_ptr<rclcpp::Node> ros_node;
            rclcpp::Subscription<sonia_common_ros2::msg::DetectionArray>::SharedPtr ai_filter_sub;

            std::vector<sonia_common_ros2::msg::Detection> _detection_array;
            std::chrono::_V2::system_clock::time_point _launch_time;
            std::chrono::duration<double> time_diff;
            int counter;
            
            BT::Expected<int> cam;
            BT::Expected<std::string> _object;
            BT::Expected<float> confidence;
            BT::Expected<int> min_detections_before_success;
            BT::Expected<int> two_objects_possible;
            BT::Expected<int> max_frame_before_failing;
            BT::Expected<float> max_time_before_failing;
            BT::Expected<float> max_depth;
    };
}