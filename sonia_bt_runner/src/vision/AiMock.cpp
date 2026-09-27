#include "sonia_bt_runner/vision/AiMock.hpp"

namespace vision {
	AiMock::AiMock(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
	: BT::StatefulActionNode(name, config), _ros_node(node), _rng(std::random_device{}()) {}

	BT::NodeStatus AiMock::onStart() {
		int camera = getInput<AI_MOCK_CAMERA_TYPE>(AI_MOCK_CAMERA).value();
		_classification = getInput<AI_MOCK_CLASSIFICATION_TYPE>(AI_MOCK_CLASSIFICATION).value();
		_number_of_detections = getInput<AI_MOCK_NUMBER_OF_DETECTIONS_TYPE>(AI_MOCK_NUMBER_OF_DETECTIONS).value();
		_number_of_messages = getInput<AI_MOCK_NUMBER_OF_MESSAGES_TYPE>(AI_MOCK_NUMBER_OF_MESSAGES).value();
		_publish_period_ms = getInput<AI_MOCK_PUBLISH_PERIOD_MS_TYPE>(AI_MOCK_PUBLISH_PERIOD_MS).value();
		_confidence_min = getInput<AI_MOCK_CONFIDENCE_MIN_TYPE>(AI_MOCK_CONFIDENCE_MIN).value();
		_confidence_max = getInput<AI_MOCK_CONFIDENCE_MAX_TYPE>(AI_MOCK_CONFIDENCE_MAX).value();
		_distance = getInput<AI_MOCK_DISTANCE_TYPE>(AI_MOCK_DISTANCE).value();
		_angle_teta = getInput<AI_MOCK_ANGLE_TETA_TYPE>(AI_MOCK_ANGLE_TETA).value();
		_distance_teta = getInput<AI_MOCK_DISTANCE_TETA_TYPE>(AI_MOCK_DISTANCE_TETA).value();
		_angle_alpha = getInput<AI_MOCK_ANGLE_ALPHA_TYPE>(AI_MOCK_ANGLE_ALPHA).value();
		_distance_beta = getInput<AI_MOCK_DISTANCE_BETA_TYPE>(AI_MOCK_DISTANCE_BETA).value();

		if (_number_of_detections < 0 || _confidence_min > _confidence_max) {
			RCLCPP_INFO(get_logger(), "Invalid AiMock inputs: check detection count and the min/max ranges.");
			return BT::NodeStatus::FAILURE;
		}

		// We publish on the same topic the AiFilter listens to, selected by the camera.
		const std::string topic = camera ? "/proc_vision/front/classif" : "/proc_vision/bottom/classif";
		_ai_mock_pub = _ros_node->create_publisher<sonia_common_ros2::msg::DetectionArray>(topic, 1);

		_published_count = 0;
		// Publish a first message immediately and arm the timer for the next ones.
		publish_detections();
		_last_publish = std::chrono::steady_clock::now();

		return BT::NodeStatus::RUNNING;
	}

	BT::NodeStatus AiMock::onRunning() {
		// Stop once we published the requested number of messages (0 means forever).
		if (_number_of_messages != 0 && _published_count >= _number_of_messages) {
			_ai_mock_pub.reset();
			return BT::NodeStatus::SUCCESS;
		}

		auto now = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_publish).count();
		if (elapsed >= _publish_period_ms) {
			publish_detections();
			_last_publish = now;
		}

		return BT::NodeStatus::RUNNING;
	}

	void AiMock::onHalted() {
		_ai_mock_pub.reset();
	}

	sonia_common_ros2::msg::Detection AiMock::build_detection() {
		std::uniform_real_distribution<float> confidence_dist(_confidence_min, _confidence_max);

		sonia_common_ros2::msg::Detection detection;
		detection.class_name = _classification;
		detection.confidence = confidence_dist(_rng);
		detection.distance = _distance;
		detection.angle_alpha = _angle_alpha;
		detection.distance_beta = _distance_beta;
		detection.angle_teta = _angle_teta;
		detection.distance_teta = _distance_teta;
		detection.frame_id = "ai_mock";

		// TODO: Not useful currently, but maybe one day 
		detection.top_left_x = 100.0;
		detection.top_left_y = 100.0;
		detection.top_right_x = 200.0;
		detection.top_right_y = 100.0;
		detection.bottom_left_x = 100.0;
		detection.bottom_left_y = 200.0;
		detection.bottom_right_x = 200.0;
		detection.bottom_right_y = 200.0;

		return detection;
	}

	void AiMock::publish_detections() {
		sonia_common_ros2::msg::DetectionArray msg;
		for (int i = 0; i < _number_of_detections; ++i) {
			msg.detected_object.push_back(build_detection());
		}

		_ai_mock_pub->publish(msg);
		_published_count++;

		RCLCPP_INFO(get_logger(), "AiMock published message %d with %d detection(s) of class '%s'", _published_count, _number_of_detections, _classification.c_str());
	}
}
