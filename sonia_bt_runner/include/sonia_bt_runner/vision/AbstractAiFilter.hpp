#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "sonia_common_ros2/msg/detection.hpp"
#include "sonia_common_ros2/msg/detection_array.hpp"

#define ABSTRACT_AI_FILTER_CAMERA "Camera"
#define ABSTRACT_AI_FILTER_CAMERA_PARAMS ABSTRACT_AI_FILTER_CAMERA, 1, "1: Front, 0: Bottom"
#define ABSTRACT_AI_FILTER_CAMERA_TYPE int

#define ABSTRACT_AI_FILTER_CONFIDENCE "Confidence"
#define ABSTRACT_AI_FILTER_CONFIDENCE_PARAMS ABSTRACT_AI_FILTER_CONFIDENCE, 0.6, "Ai confidence"
#define ABSTRACT_AI_FILTER_CONFIDENCE_TYPE float

#define ABSTRACT_AI_FILTER_MAX_DEPTH "Max_depth"
#define ABSTRACT_AI_FILTER_MAX_DEPTH_PARAMS ABSTRACT_AI_FILTER_MAX_DEPTH, 25.0, "Maximum allowed depth"
#define ABSTRACT_AI_FILTER_MAX_DEPTH_TYPE float

#define ABSTRACT_AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS "Min_detections_before_success"
#define ABSTRACT_AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_PARAMS ABSTRACT_AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS, 2, "Minumum number of frames with at least one detection before sending results. Need to be heigher than 2"
#define ABSTRACT_AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_TYPE size_t

#define ABSTRACT_AI_FILTER_MAX_FRAME_BEFORE_FAILING "Max_frame_before_failing"
#define ABSTRACT_AI_FILTER_MAX_FRAME_BEFORE_FAILING_PARAMS ABSTRACT_AI_FILTER_MAX_FRAME_BEFORE_FAILING, 0, "Maximum allowed frames before failing the object research"
#define ABSTRACT_AI_FILTER_MAX_FRAME_BEFORE_FAILING_TYPE int

#define ABSTRACT_AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC "Max_time_before_failing_sec"
#define ABSTRACT_AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_PARAMS ABSTRACT_AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC, 0, "Maximum allowed frames before failing the object research"
#define ABSTRACT_AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_TYPE float 

namespace vision {
class AbstractAiFilter : public BT::StatefulActionNode {
	public:
            AbstractAiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~AbstractAiFilter() override = default;
	    
	    static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<ABSTRACT_AI_FILTER_CAMERA_TYPE>(ABSTRACT_AI_FILTER_CAMERA_PARAMS),
                    BT::InputPort<ABSTRACT_AI_FILTER_CONFIDENCE_TYPE>(ABSTRACT_AI_FILTER_CONFIDENCE_PARAMS),
                    BT::InputPort<ABSTRACT_AI_FILTER_MAX_DEPTH_TYPE>(ABSTRACT_AI_FILTER_MAX_DEPTH_PARAMS),
                    BT::InputPort<ABSTRACT_AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_TYPE>(ABSTRACT_AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_PARAMS),
                    BT::InputPort<ABSTRACT_AI_FILTER_MAX_FRAME_BEFORE_FAILING_TYPE>(ABSTRACT_AI_FILTER_MAX_FRAME_BEFORE_FAILING_PARAMS),
                    BT::InputPort<ABSTRACT_AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_TYPE>(ABSTRACT_AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_PARAMS),
                };
            }

            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;

	protected:
	    void handle_status(BT::NodeStatus &status);
	    virtual void handle_success();
	    virtual void handle_failure();
	    virtual BT::NodeStatus get_detection_status();
	    virtual void ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg);

	    rclcpp::Logger get_logger() const noexcept { return _ros_node->get_logger(); }
	    rclcpp::Subscription<sonia_common_ros2::msg::DetectionArray>::SharedPtr ai_filter_sub() const noexcept { return _ai_filter_sub; }
	    int cam() const noexcept { return _cam; }
	    float confidence() const noexcept { return _confidence; }
	    float max_depth() const noexcept { return _max_depth; }
	    size_t detection_number_for_average() const noexcept { return _detection_number_for_average; }
	    int max_frame_before_failing() const noexcept { return _max_frame_before_failing; }
	    float max_time_before_failing() const noexcept { return _max_time_before_failing; }

	private:
            std::shared_ptr<rclcpp::Node> _ros_node;
            rclcpp::Subscription<sonia_common_ros2::msg::DetectionArray>::SharedPtr _ai_filter_sub;
	    ABSTRACT_AI_FILTER_CAMERA_TYPE _cam;
            ABSTRACT_AI_FILTER_CONFIDENCE_TYPE _confidence;
            ABSTRACT_AI_FILTER_MAX_DEPTH_TYPE _max_depth;
            ABSTRACT_AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_TYPE _detection_number_for_average;
            ABSTRACT_AI_FILTER_MAX_FRAME_BEFORE_FAILING_TYPE _max_frame_before_failing;
            ABSTRACT_AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_TYPE _max_time_before_failing;

            int _timeout_counter = 0;
	    std::chrono::_V2::system_clock::time_point _start_time;
};
}

