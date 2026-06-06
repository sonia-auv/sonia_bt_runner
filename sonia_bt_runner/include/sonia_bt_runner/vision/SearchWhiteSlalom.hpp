#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_common_ros2/msg/detection.hpp"
#include "sonia_common_ros2/msg/detection_array.hpp"

namespace vision {
class SearchWhiteSlalom : public BT::StatefulActionNode {
	public:
		SearchWhiteSlalom(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
		~SearchWhiteSlalom() override = default;

        static BT::PortsList providedPorts() {
		return {
			// Input ports
			BT::InputPort<AiDetection>("RedSlalomDetection"),
			BT::InputPort<std::string>("Side", "Left", "Left or Right"),
			BT::InputPort<float>("Confidence", 0.6f, "Minimum AI confidence"),
			BT::InputPort<int>("Min_detections_before_success", 5, "Minimum valid detections before succeeding"),
			BT::InputPort<float>("Max_time_before_failing_sec", 0.0f, "Timeout in seconds, 0 = disabled"),
			BT::InputPort<float>("Max_depth", 25.0f, "Maximum detection distance"),

			// Output port
			BT::OutputPort<AiDetection>("WhiteSlalomDetection")
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

        BT::Expected<AiDetection> _red_detection;
        BT::Expected<std::string> _side;
        BT::Expected<float> _confidence;
        BT::Expected<int> _min_detections;
        BT::Expected<float> _max_time;
        BT::Expected<float> _max_depth;
};
}
