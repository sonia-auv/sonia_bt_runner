#include "sonia_bt_runner/vision/OctogoneAiFilter.hpp"
#include "sonia_bt_runner/vision/ObjectVerification.hpp"
#include "sonia_bt_runner/vision/utils/BoxPlotToDetection.hpp"
#include "sonia_bt_runner/utils/NormalizeDetection.hpp"
#include "sonia_bt_runner/utils/CONSTANT.hpp"
#include "sonia_bt_runner/utils/AUV.hpp"
#include <cmath>
#include <cassert>

using std::placeholders::_1;
namespace vision{
    OctogoneAiFilter::OctogoneAiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : AbstractAiFilter(name, config, node)
    {
    }
    
    BT::NodeStatus OctogoneAiFilter::onStart()
    {
	    auto status = AbstractAiFilter::onStart();

	    switch (status) {
		    case BT::NodeStatus::RUNNING: 
			    
			    handle_status(status);

			    break;
	                default:
			    break;
	    }

	    return status;
    }

    BT::NodeStatus OctogoneAiFilter::get_detection_status()
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
    
    void OctogoneAiFilter::handle_success()
    {
	    RCLCPP_INFO(get_logger(), "onRunning success!!!");
	    // We need to make some selection in the image array
	    RCLCPP_INFO(get_logger(), "Getting the information because enough detection have been made : %ld detection(s)", _detection_array.size());
	    
	    // We select the detection that we want to use to compute the output
	    // boxPlotToDetection(_detection_array); // FIX_ME!!!!
	    
	    auto detected_object = detection_average();
	    
	    RCLCPP_INFO(get_logger(), "classification = %s, distance = %f, confidence = %f, angle_teta = %f, angle_alpha = %f", detected_object.classification.c_str(), detected_object.distance, detected_object.confidence, detected_object.angle_teta, detected_object.angle_alpha);
	    
	    setOutput(OCTOGONE_AI_FILTER_DETECTED_OBJECT, detected_object);

	    AbstractAiFilter::handle_success();
    }

    void OctogoneAiFilter::ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) {
	AbstractAiFilter::ai_filter_callback(msg);

        RCLCPP_INFO(get_logger(), "Start ai_filter_callback");

        for (auto msg_obj: msg.detected_object){
	    RCLCPP_INFO(get_logger(), "Received object: %s", msg_obj.class_name.c_str());
	    
            if(msg_obj.class_name == "COMPAS" || msg_obj.class_name == "LIFE PRESERVER" || msg_obj.class_name == "SOS" || msg_obj.class_name == "TOOLS")
            {
		        RCLCPP_INFO(get_logger(), "Get the wanted object: %s!!!", msg_obj.class_name.c_str());
                if(msg_obj.confidence >= confidence() && msg_obj.distance <= max_depth())
                {
                    //The detected object respect the confidence and the depth. We can put it in the filter array
                    RCLCPP_INFO(get_logger(), "Confidence and depth OK, a new object has been detected");
		            utils::normalize_detection(msg_obj, utils::get_camera_to_middle());
                    _detection_array.push_back(msg_obj);
                }
            }
        }
    }

    AiDetection OctogoneAiFilter::detection_average()
    {
        // We compute the new average of every parameter of the detection that we keep.
        AiDetection output_detection{};
        output_detection.classification = "Octogone";

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

}  // namespace vision
