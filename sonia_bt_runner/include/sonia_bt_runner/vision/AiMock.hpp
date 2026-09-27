#pragma once

#include <chrono>
#include <random>

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "sonia_common_ros2/msg/detection.hpp"
#include "sonia_common_ros2/msg/detection_array.hpp"

#define AI_MOCK_CAMERA "Camera"
#define AI_MOCK_CAMERA_PARAMS AI_MOCK_CAMERA, 1, "1: Front, 0: Bottom"
#define AI_MOCK_CAMERA_TYPE int

#define AI_MOCK_CLASSIFICATION "Classification"
#define AI_MOCK_CLASSIFICATION_PARAMS AI_MOCK_CLASSIFICATION, "SOS", "Class name of the mocked detections"
#define AI_MOCK_CLASSIFICATION_TYPE std::string

#define AI_MOCK_NUMBER_OF_DETECTIONS "Number_of_detections"
#define AI_MOCK_NUMBER_OF_DETECTIONS_PARAMS AI_MOCK_NUMBER_OF_DETECTIONS, 1, "Number of detections published in each message"
#define AI_MOCK_NUMBER_OF_DETECTIONS_TYPE int

#define AI_MOCK_NUMBER_OF_MESSAGES "Number_of_messages"
#define AI_MOCK_NUMBER_OF_MESSAGES_PARAMS AI_MOCK_NUMBER_OF_MESSAGES, 10, "Number of messages to publish before succeeding. 0 means publish forever"
#define AI_MOCK_NUMBER_OF_MESSAGES_TYPE int

#define AI_MOCK_PUBLISH_PERIOD_MS "Publish_period_ms"
#define AI_MOCK_PUBLISH_PERIOD_MS_PARAMS AI_MOCK_PUBLISH_PERIOD_MS, 100, "Delay between two published messages in milliseconds"
#define AI_MOCK_PUBLISH_PERIOD_MS_TYPE int

#define AI_MOCK_CONFIDENCE_MIN "Confidence_min"
#define AI_MOCK_CONFIDENCE_MIN_PARAMS AI_MOCK_CONFIDENCE_MIN, 0.7, "Lower bound of the random confidence"
#define AI_MOCK_CONFIDENCE_MIN_TYPE float

#define AI_MOCK_CONFIDENCE_MAX "Confidence_max"
#define AI_MOCK_CONFIDENCE_MAX_PARAMS AI_MOCK_CONFIDENCE_MAX, 1.0, "Upper bound of the random confidence"
#define AI_MOCK_CONFIDENCE_MAX_TYPE float

#define AI_MOCK_DISTANCE "Distance"
#define AI_MOCK_DISTANCE_PARAMS AI_MOCK_DISTANCE, 0.0, "Distance to the mocked object in meters"
#define AI_MOCK_DISTANCE_TYPE double

#define AI_MOCK_ANGLE_TETA "Angle_teta"
#define AI_MOCK_ANGLE_TETA_PARAMS AI_MOCK_ANGLE_TETA, 0.0, "Horizontal angle to the mocked object"
#define AI_MOCK_ANGLE_TETA_TYPE double

#define AI_MOCK_DISTANCE_TETA "Distance_teta"
#define AI_MOCK_DISTANCE_TETA_PARAMS AI_MOCK_DISTANCE_TETA, 0.0, "Distance associated with the teta angle"
#define AI_MOCK_DISTANCE_TETA_TYPE double

#define AI_MOCK_ANGLE_ALPHA "Angle_alpha"
#define AI_MOCK_ANGLE_ALPHA_PARAMS AI_MOCK_ANGLE_ALPHA, 0.0, "Vertical angle to the mocked object"
#define AI_MOCK_ANGLE_ALPHA_TYPE double

#define AI_MOCK_DISTANCE_BETA "Distance_beta"
#define AI_MOCK_DISTANCE_BETA_PARAMS AI_MOCK_DISTANCE_BETA, 0.0, "Distance associated with the alpha angle"
#define AI_MOCK_DISTANCE_BETA_TYPE double

namespace vision {
class AiMock : public BT::StatefulActionNode {
public:
	AiMock(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
	~AiMock() override = default;

	static BT::PortsList providedPorts() {
		return {
			// Inputs
			BT::InputPort<AI_MOCK_CAMERA_TYPE>(AI_MOCK_CAMERA_PARAMS),
			BT::InputPort<AI_MOCK_CLASSIFICATION_TYPE>(AI_MOCK_CLASSIFICATION_PARAMS),
			BT::InputPort<AI_MOCK_NUMBER_OF_DETECTIONS_TYPE>(AI_MOCK_NUMBER_OF_DETECTIONS_PARAMS),
			BT::InputPort<AI_MOCK_NUMBER_OF_MESSAGES_TYPE>(AI_MOCK_NUMBER_OF_MESSAGES_PARAMS),
			BT::InputPort<AI_MOCK_PUBLISH_PERIOD_MS_TYPE>(AI_MOCK_PUBLISH_PERIOD_MS_PARAMS),
			BT::InputPort<AI_MOCK_CONFIDENCE_MIN_TYPE>(AI_MOCK_CONFIDENCE_MIN_PARAMS),
			BT::InputPort<AI_MOCK_CONFIDENCE_MAX_TYPE>(AI_MOCK_CONFIDENCE_MAX_PARAMS),
			BT::InputPort<AI_MOCK_DISTANCE_TYPE>(AI_MOCK_DISTANCE_PARAMS),
			BT::InputPort<AI_MOCK_ANGLE_TETA_TYPE>(AI_MOCK_ANGLE_TETA_PARAMS),
			BT::InputPort<AI_MOCK_DISTANCE_TETA_TYPE>(AI_MOCK_DISTANCE_TETA_PARAMS),
			BT::InputPort<AI_MOCK_ANGLE_ALPHA_TYPE>(AI_MOCK_ANGLE_ALPHA_PARAMS),
			BT::InputPort<AI_MOCK_DISTANCE_BETA_TYPE>(AI_MOCK_DISTANCE_BETA_PARAMS),
		};
	}

	BT::NodeStatus onStart() override;
	BT::NodeStatus onRunning() override;
	void onHalted() override;

private:
	sonia_common_ros2::msg::Detection build_detection();
	void publish_detections();

	rclcpp::Logger get_logger() const noexcept { return _ros_node->get_logger(); }

	std::shared_ptr<rclcpp::Node> _ros_node;
	rclcpp::Publisher<sonia_common_ros2::msg::DetectionArray>::SharedPtr _ai_mock_pub;

	AI_MOCK_CLASSIFICATION_TYPE _classification;
	AI_MOCK_NUMBER_OF_DETECTIONS_TYPE _number_of_detections;
	AI_MOCK_NUMBER_OF_MESSAGES_TYPE _number_of_messages;
	AI_MOCK_PUBLISH_PERIOD_MS_TYPE _publish_period_ms;
	AI_MOCK_CONFIDENCE_MIN_TYPE _confidence_min;
	AI_MOCK_CONFIDENCE_MAX_TYPE _confidence_max;
	AI_MOCK_DISTANCE_TYPE _distance;
	AI_MOCK_ANGLE_TETA_TYPE _angle_teta;
	AI_MOCK_DISTANCE_TETA_TYPE _distance_teta;
	AI_MOCK_ANGLE_ALPHA_TYPE _angle_alpha;
	AI_MOCK_DISTANCE_BETA_TYPE _distance_beta;

	int _published_count = 0;
	std::chrono::steady_clock::time_point _last_publish;

	std::mt19937 _rng;
};
}
