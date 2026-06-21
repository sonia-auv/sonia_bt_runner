#include "sonia_bt_runner/vision/AbstractAiFilter.hpp"

using std::placeholders::_1;
namespace vision {
	AbstractAiFilter::AbstractAiFilter(const std::string &name,
			 const BT::NodeConfig &config,
			 std::shared_ptr<rclcpp::Node> node) :
		BT::StatefulActionNode(name, config), _ros_node(node) {}

	BT::NodeStatus AbstractAiFilter::onStart() {
		_cam = getInput<ABSTRACT_AI_FILTER_CAMERA_TYPE>(ABSTRACT_AI_FILTER_CAMERA).value();
		_confidence = getInput<ABSTRACT_AI_FILTER_CONFIDENCE_TYPE>(ABSTRACT_AI_FILTER_CONFIDENCE).value();
		_max_depth = getInput<ABSTRACT_AI_FILTER_MAX_DEPTH_TYPE>(ABSTRACT_AI_FILTER_MAX_DEPTH).value();
		_detection_number_for_average = getInput<ABSTRACT_AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_TYPE>(ABSTRACT_AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS).value();
		
		_max_frame_before_failing = getInput<ABSTRACT_AI_FILTER_MAX_FRAME_BEFORE_FAILING_TYPE>(ABSTRACT_AI_FILTER_MAX_FRAME_BEFORE_FAILING).value();
		_max_time_before_failing = getInput<ABSTRACT_AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_TYPE>(ABSTRACT_AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC).value();

		_start_time = std::chrono::system_clock::now();
		
		// We create the subscriber to gather the information
		if(_cam)
			_ai_filter_sub = _ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/front/classif", 1, std::bind(&AbstractAiFilter::ai_filter_callback, this, _1));
		else
			_ai_filter_sub = _ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/bottom/classif", 1, std::bind(&AbstractAiFilter::ai_filter_callback, this, _1));

		auto status = BT::NodeStatus::RUNNING;
		
		if (_detection_number_for_average <= 1) {
			RCLCPP_INFO(_ros_node->get_logger(), "You have to set the Min_detections_before_success parameter to more than 1.");
			status = BT::NodeStatus::FAILURE;
		}
		
		handle_status(status);

		return status;
	}
	
	BT::NodeStatus AbstractAiFilter::get_detection_status()
	{
		auto status = BT::NodeStatus::RUNNING;
		std::chrono::duration<double> diff = std::chrono::system_clock::now() - _start_time;
		float time_diff = diff.count();

		if (_max_frame_before_failing != 0 && _timeout_counter > _max_frame_before_failing) {
			RCLCPP_INFO(_ros_node->get_logger(), "max frame = %d, We don't find what we are looking for.", _max_frame_before_failing);

			status = BT::NodeStatus::FAILURE;
		} else if (_max_time_before_failing != 0.0f && time_diff > _max_time_before_failing) {
			RCLCPP_INFO(_ros_node->get_logger(), "max time = %f, We don't find what we are looking for.", _max_time_before_failing);

			status = BT::NodeStatus::FAILURE;
		}

		handle_status(status);

		return status;
	}

	BT::NodeStatus AbstractAiFilter::onRunning() {
		RCLCPP_INFO(_ros_node->get_logger(), "onRunning");
		
		auto detection_status = get_detection_status();	

		handle_status(detection_status);

		return detection_status;
	}

        void AbstractAiFilter::onHalted()
	{
		/* Unused */
	}

	void AbstractAiFilter::handle_success()
	{
		_ai_filter_sub.reset();
	}
	
	void AbstractAiFilter::handle_failure()
	{
		_ai_filter_sub.reset();
	}
	
	void AbstractAiFilter::handle_status(BT::NodeStatus &status) {
	    switch (status) {
			case BT::NodeStatus::SUCCESS:
				handle_success();

				break;
			case BT::NodeStatus::FAILURE:
				handle_failure();

				break;
			default:
				break;
		}
    }
	
    void AbstractAiFilter::ai_filter_callback([[maybe_unused]] const sonia_common_ros2::msg::DetectionArray &msg)
    {
	    _timeout_counter++;
	    RCLCPP_INFO(_ros_node->get_logger(), "Start ai_filter_callback");
    }
}
