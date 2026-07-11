#include "sonia_bt_runner/vision/SearchClosestSlalom.hpp"
#include "sonia_bt_runner/vision/ObjectVerification.hpp"
#include "sonia_bt_runner/vision/utils/BoxPlotToDetection.hpp"
#include "sonia_bt_runner/utils/NormalizeDetection.hpp"

#include <optional>

using std::placeholders::_1;

namespace vision{
    SearchClosestSlalom::SearchClosestSlalom(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : AbstractAiFilter(name, config, node)
    {
    }

    BT::NodeStatus SearchClosestSlalom::onStart()
    {
        auto status = AbstractAiFilter::onStart();

	    switch (status) {
		    case BT::NodeStatus::RUNNING: {
			    _object_class = getInput<SEARCH_CLOSEST_SLALOM_OBJECT_CLASS_TYPE>(SEARCH_CLOSEST_SLALOM_OBJECT_CLASS).value();
                _actual_angle = getInput<SEARCH_CLOSEST_SLALOM_ACTUAL_ANGLE_TYPE>(SEARCH_CLOSEST_SLALOM_ACTUAL_ANGLE).value();

                if (!getInput<SEARCH_CLOSEST_SLALOM_CLOSEST_OBJECT_ANGLE_TYPE>(SEARCH_CLOSEST_SLALOM_CLOSEST_OBJECT_ANGLE)) {
                    _closest_object_angle = 0.0f;
                } else {
                    _closest_object_angle = getInput<SEARCH_CLOSEST_SLALOM_CLOSEST_OBJECT_ANGLE_TYPE>(SEARCH_CLOSEST_SLALOM_CLOSEST_OBJECT_ANGLE).value();
                }

		        auto detected_object = getInput<SEARCH_CLOSEST_SLALOM_CLOSEST_DETECTED_OBJECT_TYPE>(SEARCH_CLOSEST_SLALOM_CLOSEST_DETECTED_OBJECT);

                if (detected_object) {
                    _closest_object_detected = detected_object.value(); 
                } else {
                    _closest_object_detected = {};
                }

			    // We verify if the object is valid
			    if (!verifyObject(_object_class)) {
				    RCLCPP_INFO(get_logger(), "The detected object is not a valid name of type of detection. Syntax error");
				    status = BT::NodeStatus::FAILURE;
			    }

			    if (_object_class != "RED_SLALOM" && _object_class != "WHITE_SLALOM") {
				    status = BT::NodeStatus::FAILURE;
			    }

			    _side = getInput<SEARCH_CLOSEST_SLALOM_SIDE_TYPE>(SEARCH_CLOSEST_SLALOM_SIDE).value();

			    handle_status(status);

			    break;
		    }
	            default:
			    break;
	    }

	    return status;
    }

    BT::NodeStatus SearchClosestSlalom::get_detection_status()
    {
        switch (AbstractAiFilter::get_detection_status()) {
		    case BT::NodeStatus::FAILURE:
			    return BT::NodeStatus::FAILURE;
		    case BT::NodeStatus::RUNNING:
			    if (_detection_array.size() < detection_number_for_average()) {
				    RCLCPP_INFO(get_logger(), "We don't have yet the number of detection we want");
				    return BT::NodeStatus::RUNNING;
			    }
			    return BT::NodeStatus::SUCCESS;
		    default:
			    assert(0 && "Not expected status");
	    }
    }

    void SearchClosestSlalom::handle_success()
    {
	    RCLCPP_INFO(get_logger(), "onRunning success!!!");
	    // We need to make some selection in the image array
	    RCLCPP_INFO(get_logger(), "Getting the information because enough detection have been made : %ld detection(s)", _detection_array.size());
	    
	    // We select the detection that we want to use to compute the output
	    // boxPlotToDetection(_detection_array); // FIX_ME!!!!
	    
	    auto detected_object = detection_average();
	    auto detected_object_fn = [&]() {
		    // A new detection has been detected much further than the last one
		    _closest_object_detected = detected_object;
		    _closest_object_angle = _actual_angle;
		    
		    RCLCPP_INFO(get_logger(), "New closest detection classification = %s, distance = %f, confidence = %f, angle_teta = %f, angle_alpha = %f",
				    _closest_object_detected.value().classification.c_str(),
        _closest_object_detected.value().distance,
        _closest_object_detected.value().confidence,
        _closest_object_detected.value().angle_teta,
        _closest_object_detected.value().angle_alpha);
	    
	    setOutput(SEARCH_CLOSEST_SLALOM_CLOSEST_DETECTED_OBJECT, _closest_object_detected.value());
        setOutput(SEARCH_CLOSEST_SLALOM_CLOSEST_OBJECT_ANGLE, _closest_object_angle);
	    };

	    if (_closest_object_detected.has_value()) {
		    if (detected_object.distance < _closest_object_detected.value().distance) {
			    detected_object_fn();
		    }
	    } else {
		    detected_object_fn();
	    } 

	    AbstractAiFilter::handle_success();
    }

    AiDetection SearchClosestSlalom::detection_average()
    {
        // We compute the new average of every parameter of the detection that we keep.
        AiDetection output_detection{};
        output_detection.classification = _object_class;

	    assert(_detection_array.size() != 0 && "Something wrong in the code");

        for(auto detection : _detection_array)
        {
            output_detection.distance += detection.distance;
            output_detection.confidence += detection.confidence;
            output_detection.angle_teta += detection.angle_teta;
            output_detection.angle_alpha += detection.angle_alpha;
            output_detection.distance_teta += detection.distance_teta;
            output_detection.distance_beta += detection.distance_beta;
        }
        output_detection.distance /= _detection_array.size();
        output_detection.confidence /= _detection_array.size();
        output_detection.angle_teta /= _detection_array.size();
        output_detection.angle_alpha /= _detection_array.size();
        output_detection.distance_teta /= _detection_array.size();
        output_detection.distance_beta /= _detection_array.size();

        return output_detection;
    }

    void SearchClosestSlalom::ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg)
    {
        std::optional<sonia_common_ros2::msg::Detection> closest_detection;
        AbstractAiFilter::ai_filter_callback(msg);
    
        RCLCPP_INFO(get_logger(), "Start search_closest_slalom_callback");

        for (auto msg_obj: msg.detected_object){
            RCLCPP_INFO(get_logger(), "Received object: %s", msg_obj.class_name.c_str());
            if(msg_obj.class_name.compare(_object_class) == 0)
            {
                RCLCPP_INFO(get_logger(), "Get the wanted object: %s!!!", msg_obj.class_name.c_str());

                if (_object_class == "WHITE_SLALOM") {
                    if ((_actual_angle - msg_obj.angle_alpha < 0 && _side == "Right") 
                     || (_actual_angle - msg_obj.angle_alpha > 0 && _side == "Left")) {
                        RCLCPP_INFO(get_logger(), "The white slalom is on the wrong side");
                        continue;
                    }
                }

		utils::normalize_detection(msg_obj);

                if(msg_obj.confidence >= confidence() && msg_obj.distance <= max_depth())
                {
                    RCLCPP_INFO(get_logger(), "Confidence and depth OK");
                    if (closest_detection.has_value()) {
                        if (closest_detection.value().distance > msg_obj.distance) {
                            closest_detection = msg_obj;
                            RCLCPP_INFO(get_logger(), "We get a new closest object");
                        }
                    } else {
                        closest_detection = msg_obj;
                        RCLCPP_INFO(get_logger(), "We get a new closest object");
                    }
                }
            }
        }
        if (closest_detection.has_value()) {
            _detection_array.push_back(closest_detection.value());
        }
    }
}
