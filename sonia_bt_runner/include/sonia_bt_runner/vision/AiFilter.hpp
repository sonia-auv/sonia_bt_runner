#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_common_ros2/msg/detection.hpp"
#include "sonia_common_ros2/msg/detection_array.hpp"


namespace vision{
    class AiFilter: public BT::StatefulActionNode{
        public:
            AiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~AiFilter() override = default;
            static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<int>("Camera", 1, "1: Front, 0: Bottom"),
                    BT::InputPort<std::string>("Object_class", "Searched object"),
                    BT::InputPort<float>("Confidence", 0.6, "Ai confidence"),
                    BT::InputPort<float>("Max_depth", 25.0, "Maximum allowed depth"),
                    BT::InputPort<int>("Min_detections_before_success", 2, "Minumum number of frames with at least one detection before sending results. Need to be heigher than 2"),

                    BT::InputPort<int>("Max_frame_before_failing", 0, "Maximum allowed frames before failing the object research"),
                    BT::InputPort<float>("Max_time_before_failing_sec", 0.0, "Maximum allowed time in ms before failing the object research"),

                    // Outputs
                    BT::OutputPort<AiDetection>("Detected_object")
                };
            }

            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;

        private:
            rclcpp::Subscription<sonia_common_ros2::msg::DetectionArray>::SharedPtr _ai_filter_sub;
            // std::chrono::_V2::system_clock::time_point _launch_time;
            // float _time_diff;
            
        protected:
            std::shared_ptr<rclcpp::Node> _ros_node;
            std::string _object_filter;
            float _confidence_filter;
            float _max_depth_filter;
            std::vector<sonia_common_ros2::msg::Detection> _detection_array;
            int _timout_counter = 0;
			std::chrono::_V2::system_clock::time_point _start_time;
            BT::Expected<int> _cam;
            BT::Expected<std::string> _object;
            BT::Expected<float> _confidence;
            BT::Expected<float> _max_depth;
            BT::Expected<int> _detection_number_for_average;
            BT::Expected<int> _max_frame_before_failing;
            BT::Expected<float> _max_time_before_failing;
            
            virtual void ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg);
            BT::NodeStatus get_detection_status(const std::string& object, const float confidence, const float max_depth);
            void applicate_box_plot_to_detections();
            AiDetection detection_average();
            
            // void applicate_confidence_interval_to_detections();

    };
}
