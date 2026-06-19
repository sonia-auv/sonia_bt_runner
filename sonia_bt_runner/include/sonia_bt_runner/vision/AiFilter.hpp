#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_common_ros2/msg/detection.hpp"
#include "sonia_common_ros2/msg/detection_array.hpp"

#define AI_FILTER_CAMERA_NAME "Camera"
#define AI_FILTER_CAMERA_TYPE int
#define AI_FILTER_OBJECT_CLASS_NAME "Object_class"
#define AI_FILTER_OBJECT_CLASS_TYPE std::string
#define AI_FILTER_CONFIDENCE_NAME "Confidence"
#define AI_FILTER_CONFIDENCE_TYPE float
#define AI_FILTER_MAX_DEPTH_NAME "Max_depth"
#define AI_FILTER_MAX_DEPTH_TYPE float
#define AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_NAME "Min_detections_before_success"
#define AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_TYPE int
#define AI_FILTER_MAX_FRAME_BEFORE_FAILING_NAME "Max_frame_before_failing"
#define AI_FILTER_MAX_FRAME_BEFORE_FAILING_TYPE int
#define AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_NAME "Max_time_before_failing_sec"
#define AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_TYPE float
#define AI_FILTER_DETECTED_OBJECT_NAME "Detected_object"
#define AI_FILTER_DETECTED_OBJECT_TYPE AiDetection

namespace vision{
    class AiFilter: public BT::StatefulActionNode{
        public:
            AiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~AiFilter() override = default;
            static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<AI_FILTER_CAMERA_TYPE>(AI_FILTER_CAMERA_NAME, 1, "1: Front, 0: Bottom"),
                    BT::InputPort<AI_FILTER_OBJECT_CLASS_TYPE>(AI_FILTER_OBJECT_CLASS_NAME, "Searched object"),
                    BT::InputPort<AI_FILTER_CONFIDENCE_TYPE>(AI_FILTER_CONFIDENCE_NAME, 0.6, "Ai confidence"),
                    BT::InputPort<AI_FILTER_MAX_DEPTH_TYPE>(AI_FILTER_MAX_DEPTH_NAME, 25.0, "Maximum allowed depth"),
                    BT::InputPort<AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_TYPE>(AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_NAME, 2, "Minumum number of frames with at least one detection before sending results. Need to be heigher than 2"),

                    BT::InputPort<AI_FILTER_MAX_FRAME_BEFORE_FAILING_TYPE>(AI_FILTER_MAX_FRAME_BEFORE_FAILING_NAME, 0, "Maximum allowed frames before failing the object research"),
                    BT::InputPort<AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_TYPE>(AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_NAME, 0.0, "Maximum allowed time in ms before failing the object research"),

                    // Outputs
                    BT::OutputPort<AI_FILTER_DETECTED_OBJECT_TYPE>(AI_FILTER_DETECTED_OBJECT_NAME)
                };
            }

            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;

        private:
            rclcpp::Subscription<sonia_common_ros2::msg::DetectionArray>::SharedPtr _ai_filter_sub;
            std::chrono::_V2::system_clock::time_point _launch_time;
            
        protected:
            std::shared_ptr<rclcpp::Node> _ros_node;
            std::string _object_filter;
            float _confidence_filter;
            float _max_depth_filter;
            std::vector<sonia_common_ros2::msg::Detection> _detection_array;
            int _timout_counter;
			std::chrono::_V2::system_clock::time_point _start_time;
            BT::Expected<AI_FILTER_CAMERA_TYPE> _cam;
            BT::Expected<AI_FILTER_OBJECT_CLASS_TYPE> _object;
            BT::Expected<AI_FILTER_CONFIDENCE_TYPE> _confidence;
            BT::Expected<AI_FILTER_MAX_DEPTH_TYPE> _max_depth;
            BT::Expected<AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_TYPE> _detection_number_for_average;
            BT::Expected<AI_FILTER_MAX_FRAME_BEFORE_FAILING_TYPE> _max_frame_before_failing;
            BT::Expected<AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_TYPE> _max_time_before_failing;
            
            virtual void ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg);
            virtual void stock_input_parameters();
            bool set_filter_parameter(const std::string& object, const float confidence, const float max_depth);
            virtual bool initial_condition_verification();
            void initialize_subscriber();
            void delete_subscriber();
            virtual BT::NodeStatus get_detection_status();
            virtual void applicate_box_plot_to_detections();
            virtual void setting_output();
            AI_FILTER_DETECTED_OBJECT_TYPE detection_average(const std::vector<sonia_common_ros2::msg::Detection>& detection_array);
    };
}
