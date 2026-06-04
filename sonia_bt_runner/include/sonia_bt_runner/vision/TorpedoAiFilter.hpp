#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"
#include "sonia_common_ros2/msg/detection.hpp"
#include "sonia_common_ros2/msg/detection_array.hpp"


namespace vision {
    class TorpedoAi : public BT::StatefulActionNode {
        public:
            TorpedoAi(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~TorpedoAi() override = default;
            static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<std::string>("Object_class", "Searched object"),
                    BT::InputPort<float>("Confidence", 0.6, "Ai _confidence"),
                    BT::InputPort<int>("Min_detections_before_success", 2, "Minumum number of frames with at least one detection before sending results. Need to be heigher than 2"),
                    BT::InputPort<int>("Max_frame_before_failing", 0, "Maximum allowed frames before failing the object research"),
                    BT::InputPort<float>("Max_time_before_failing_sec", 0.0, "Maximum allowed time in ms before failing the object research"),
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

            std::shared_ptr<rclcpp::Node> _ros_node;
            rclcpp::Subscription<sonia_common_ros2::msg::DetectionArray>::SharedPtr _ai_filter_sub;
            std::vector<sonia_common_ros2::msg::Detection> _detection_array;
            std::chrono::_V2::system_clock::time_point _launch_time;
            float _time_diff;
            int _timout_counter;
            bool _detecting_target;
            sonia_common_ros2::msg::Detection _main_object;
            
            BT::Expected<std::string> _object;
            BT::Expected<float> _confidence;
            BT::Expected<int> _min_detections_before_success;
            BT::Expected<int> _max_frame_before_failing;
            BT::Expected<float> _max_time_before_failing;
            BT::Expected<float> _max_depth;
    };
}