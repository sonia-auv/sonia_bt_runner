#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_common_ros2/msg/detection.hpp"
#include "sonia_common_ros2/msg/detection_array.hpp"

#define SEARCH_WHITE_SLALOM_RED_SLALOM_DETECTION "RedSlalomDetection"
#define SEARCH_WHITE_SLALOM_RED_SLALOM_DETECTION_TYPE AiDetection

#define SEARCH_WHITE_SLALOM_SIDE "Side"
#define SEARCH_WHITE_SLALOM_SIDE_PARAMS SEARCH_WHITE_SLALOM_SIDE, "Left", "Left or Right"
#define SEARCH_WHITE_SLALOM_SIDE_TYPE std::string

#define SEARCH_WHITE_SLALOM_CONFIDENCE "Confidence"
#define SEARCH_WHITE_SLALOM_CONFIDENCE_PARAMS SEARCH_WHITE_SLALOM_CONFIDENCE, 0.6f, "Minimum AI confidence"
#define SEARCH_WHITE_SLALOM_CONFIDENCE_TYPE float

#define SEARCH_WHITE_SLALOM_MIN_DETECTIONS_BEFORE_SUCCESS "Min_detections_before_success"
#define SEARCH_WHITE_SLALOM_MIN_DETECTIONS_BEFORE_SUCCESS_PARAMS SEARCH_WHITE_SLALOM_MIN_DETECTIONS_BEFORE_SUCCESS, 5, "Minimum valid detections before succeeding"
#define SEARCH_WHITE_SLALOM_MIN_DETECTIONS_BEFORE_SUCCESS_TYPE int

#define SEARCH_WHITE_SLALOM_MAX_TIME_BEFORE_FAILING_SEC "Max_time_before_failing_sec"
#define SEARCH_WHITE_SLALOM_MAX_TIME_BEFORE_FAILING_SEC_PARAMS SEARCH_WHITE_SLALOM_MAX_TIME_BEFORE_FAILING_SEC, 0.0f, "Timeout in seconds, 0 = disabled"
#define SEARCH_WHITE_SLALOM_MAX_TIME_BEFORE_FAILING_SEC_TYPE float

#define SEARCH_WHITE_SLALOM_MAX_DEPTH "Max_depth"
#define SEARCH_WHITE_SLALOM_MAX_DEPTH_PARAMS SEARCH_WHITE_SLALOM_MAX_DEPTH, 25.0f, "Maximum detection distance"
#define SEARCH_WHITE_SLALOM_MAX_DEPTH_TYPE float

#define SEARCH_WHITE_SLALOM_WHITE_SLALOM_DETECTION "WhiteSlalomDetection"
#define SEARCH_WHITE_SLALOM_WHITE_SLALOM_DETECTION_TYPE AiDetection

namespace vision {
class SearchWhiteSlalom : public BT::StatefulActionNode {
	public:
		SearchWhiteSlalom(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
		~SearchWhiteSlalom() override = default;

        static BT::PortsList providedPorts() {
		return {
			// Input ports
			BT::InputPort<SEARCH_WHITE_SLALOM_RED_SLALOM_DETECTION_TYPE>(SEARCH_WHITE_SLALOM_RED_SLALOM_DETECTION),
			BT::InputPort<SEARCH_WHITE_SLALOM_SIDE_TYPE>(SEARCH_WHITE_SLALOM_SIDE_PARAMS),
			BT::InputPort<SEARCH_WHITE_SLALOM_CONFIDENCE_TYPE>(SEARCH_WHITE_SLALOM_CONFIDENCE_PARAMS),
			BT::InputPort<SEARCH_WHITE_SLALOM_MIN_DETECTIONS_BEFORE_SUCCESS_TYPE>(SEARCH_WHITE_SLALOM_MIN_DETECTIONS_BEFORE_SUCCESS_PARAMS),
			BT::InputPort<SEARCH_WHITE_SLALOM_MAX_TIME_BEFORE_FAILING_SEC_TYPE>(SEARCH_WHITE_SLALOM_MAX_TIME_BEFORE_FAILING_SEC_PARAMS),
			BT::InputPort<SEARCH_WHITE_SLALOM_MAX_DEPTH_TYPE>(SEARCH_WHITE_SLALOM_MAX_DEPTH_PARAMS),

			// Output port
			BT::OutputPort<SEARCH_WHITE_SLALOM_WHITE_SLALOM_DETECTION_TYPE>(SEARCH_WHITE_SLALOM_WHITE_SLALOM_DETECTION)
		};
	    }

	    BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

	private:
        void detection_callback(const sonia_common_ros2::msg::DetectionArray &msg);

        std::shared_ptr<rclcpp::Node> _ros_node;
        rclcpp::Subscription<sonia_common_ros2::msg::DetectionArray>::SharedPtr _sub;
        std::vector<sonia_common_ros2::msg::Detection> _valid_detections;
        std::chrono::_V2::system_clock::time_point _launch_time;
        float _time_diff;

        BT::Expected<SEARCH_WHITE_SLALOM_RED_SLALOM_DETECTION_TYPE> _red_detection;
        BT::Expected<SEARCH_WHITE_SLALOM_SIDE_TYPE> _side;
        BT::Expected<SEARCH_WHITE_SLALOM_CONFIDENCE_TYPE> _confidence;
        BT::Expected<SEARCH_WHITE_SLALOM_MIN_DETECTIONS_BEFORE_SUCCESS_TYPE> _min_detections;
        BT::Expected<SEARCH_WHITE_SLALOM_MAX_TIME_BEFORE_FAILING_SEC_TYPE> _max_time;
        BT::Expected<SEARCH_WHITE_SLALOM_MAX_DEPTH_TYPE> _max_depth;
};
}
