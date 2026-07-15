#include "sonia_bt_runner/vision/SlalomScan.hpp"
#include "sonia_bt_runner/utils/NormalizeDetection.hpp"
#include "sonia_bt_runner/utils/CONSTANT.hpp"

#include <cmath>

#define DEG_TO_RAD (M_PI/180.0f)

using std::placeholders::_1;

namespace vision{
    SlalomScan::SlalomScan(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : AbstractAiFilter(name, config, node)
    {
    }

    BT::NodeStatus SlalomScan::onStart()
    {
        auto status = AbstractAiFilter::onStart();

	    switch (status) {
		    case BT::NodeStatus::RUNNING: {
                _actual_angle = getInput<SLALOM_SCAN_ACTUAL_ANGLE_TYPE>(SLALOM_SCAN_ACTUAL_ANGLE).value();
                _invert_camera_angle = getInput<SLALOM_SCAN_INVERT_CAMERA_ANGLE_TYPE>(SLALOM_SCAN_INVERT_CAMERA_ANGLE).value();
                _frames_with_detection = 0;

                auto points = getInput<SLALOM_SCAN_POINTS_TYPE>(SLALOM_SCAN_POINTS);

                if (points) {
                    _points = points.value();
                } else {
                    _points = {};
                }

			    break;
		    }
	            default:
			    break;
	    }

	    return status;
    }

    BT::NodeStatus SlalomScan::get_detection_status()
    {
        switch (AbstractAiFilter::get_detection_status()) {
		    case BT::NodeStatus::FAILURE:
			    return BT::NodeStatus::FAILURE;
		    case BT::NodeStatus::RUNNING:
			    if (_frames_with_detection < detection_number_for_average()) {
				    RCLCPP_INFO(get_logger(), "We don't have yet the number of detection we want");
				    return BT::NodeStatus::RUNNING;
			    }
			    return BT::NodeStatus::SUCCESS;
		    default:
			    assert(0 && "Not expected status");
	    }
    }

    void SlalomScan::flush_points()
    {
	    RCLCPP_INFO(get_logger(), "Flushing %ld slalom point(s) at angle %f", _points.points.size(), _actual_angle);
	    setOutput(SLALOM_SCAN_POINTS, _points);
    }

    void SlalomScan::handle_success()
    {
	    flush_points();
	    AbstractAiFilter::handle_success();
    }

    void SlalomScan::handle_failure()
    {
	    // A timeout only means that not enough frames had a slalom in view, the
	    // points gathered before the timeout are still valid
	    flush_points();
	    AbstractAiFilter::handle_failure();
    }

    void SlalomScan::ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg)
    {
        AbstractAiFilter::ai_filter_callback(msg);

        RCLCPP_INFO(get_logger(), "Start slalom_scan_callback");

        bool frame_has_detection = false;

        for (auto msg_obj: msg.detected_object){
            if (msg_obj.class_name.compare("RED_SLALOM") != 0 && msg_obj.class_name.compare("WHITE_SLALOM") != 0)
            {
                continue;
            }

            RCLCPP_INFO(get_logger(), "Get a slalom detection: %s", msg_obj.class_name.c_str());

	    utils::normalize_detection(msg_obj, utils::get_camera_to_middle());

            if(msg_obj.confidence >= confidence() && msg_obj.distance <= max_depth())
            {
                // FaceTheObject rotates by -angle_alpha to face a detection, so the
                // absolute bearing of a detection is the sub angle minus angle_alpha
                float camera_angle = _invert_camera_angle ? -msg_obj.angle_alpha : msg_obj.angle_alpha;
                float bearing = (_actual_angle - camera_angle) * DEG_TO_RAD;

                SlalomPoint point{};
                point.x = msg_obj.distance * std::cos(bearing);
                point.y = msg_obj.distance * std::sin(bearing);
                point.classification = msg_obj.class_name;
                point.confidence = msg_obj.confidence;
                _points.points.push_back(point);

                frame_has_detection = true;

                RCLCPP_INFO(get_logger(), "New slalom point %s at x = %f, y = %f (distance = %f, bearing = %f)",
                            point.classification.c_str(), point.x, point.y, msg_obj.distance, bearing / DEG_TO_RAD);
            }
        }

        if (frame_has_detection) {
            _frames_with_detection++;
        }
    }
}
