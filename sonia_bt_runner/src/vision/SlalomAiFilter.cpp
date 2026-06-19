#include "sonia_bt_runner/vision/SlalomAiFilter.hpp"

using std::placeholders::_1;

namespace vision {

    SlalomAiFilter::SlalomAiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : AiFilter(name, config, node)
{}


    BT::NodeStatus SlalomAiFilter::onStart()
    {
        RCLCPP_INFO(_ros_node->get_logger(), "SlalomAiFilter onStart");

        stock_input_parameters();

        if (!initial_condition_verification()) {
            return BT::NodeStatus::FAILURE;
        }

        initialize_subscriber();
        RCLCPP_INFO(_ros_node->get_logger(), "SlalomAiFilter onStart finished");
        return BT::NodeStatus::RUNNING;

    }

    void SlalomAiFilter::ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg)
    {
        RCLCPP_INFO(_ros_node->get_logger(), "SlalomAiFilter callback running");
        std::optional<sonia_common_ros2::msg::Detection> closest_red_slalom;
        std::vector<sonia_common_ros2::msg::Detection> whites_slaloms;
        sonia_common_ros2::msg::Detection choosen_white_slalom;
        float closest_distance;
        size_t closest_index;

        for (auto msg_obj : msg.detected_object) {
            // We first detect the closest red slalom on the image
            if (msg_obj.class_name.compare("COMPASS") == 0
             && msg_obj.confidence >= _confidence_filter
             && msg_obj.distance <= _max_depth_filter
             && (!closest_red_slalom || closest_red_slalom.value().distance >= msg_obj.distance))
            {
                closest_red_slalom = msg_obj;
            }

            // We detect the whites slaloms
            if (msg_obj.class_name.compare("TOOLS") == 0
                && msg_obj.confidence >= _confidence_filter
                && msg_obj.distance <= _max_depth_filter)
            {
                whites_slaloms.push_back(msg_obj);
            }
        }
        
        // We verify if we have both a RED_SLALOM and at least one WHITE_SLALOM
        if (!closest_red_slalom.has_value() || !whites_slaloms.size()) {
            return;
        }

        // We push_back the red and white slalom
        _detection_array.push_back(closest_red_slalom.value());
        
        // If only one slalom is present, We keep it
        if (whites_slaloms.size() == 1) {
            _detection_array_white_slalom.push_back(whites_slaloms[0]);
            return;
        }

        // We first get the closest white slalom
        closest_distance = whites_slaloms[0].distance;
        for (size_t i{1}; i < whites_slaloms.size(); ++i){
            if (whites_slaloms[i].distance < closest_distance) {
                closest_index = i;
                closest_distance = whites_slaloms[i].distance;
            }
        }

        // We verify if an author slalom has approximatively the same distance as the closest one
        // In this case, we keep the slalom on the {side} choosen
        for (size_t i{1}; i < whites_slaloms.size(); ++i) {
            if (closest_index != i && whites_slaloms[i].distance < closest_distance + 0.5) {
                if (whites_slaloms[i].distance_teta > whites_slaloms[closest_index].distance_teta) {
                    if (_side.value() == "right") {
                        _detection_array_white_slalom.push_back(whites_slaloms[i]);
                    } else {
                        _detection_array_white_slalom.push_back(whites_slaloms[closest_index]);
                    }
                } else {
                    if (_side.value() == "left") {
                        _detection_array_white_slalom.push_back(whites_slaloms[i]);
                    } else {
                        _detection_array_white_slalom.push_back(whites_slaloms[closest_index]);
                    }
                }
                return;
            }
        }

        // Every detection is to far, so we keep the closest slalom
        _detection_array_white_slalom.push_back(whites_slaloms[closest_index]);

    }

    void SlalomAiFilter::stock_input_parameters()
    {
        // We go get the information in the behavior tree
        RCLCPP_INFO(_ros_node->get_logger(), "SlalomAiFilter side");
        _side = getInput<SLALOM_AI_FILTER_SIDE_TYPE>(SLALOM_AI_FILTER_SIDE_NAME);
        RCLCPP_INFO(_ros_node->get_logger(), "SlalomAiFilter camera");
        _cam = getInput<AI_FILTER_CAMERA_PARAM_TYPE>(AI_FILTER_CAMERA_PARAM_NAME);
        RCLCPP_INFO(_ros_node->get_logger(), "SlalomAiFilter confidence");
        _confidence = getInput<AI_FILTER_CONFIDENCE_PARAM_TYPE>(AI_FILTER_CONFIDENCE_PARAM_NAME);
        RCLCPP_INFO(_ros_node->get_logger(), "SlalomAiFilter max_depth");
        _max_depth = getInput<AI_FILTER_MAX_DEPTH_PARAM_TYPE>(AI_FILTER_MAX_DEPTH_PARAM_NAME);
        RCLCPP_INFO(_ros_node->get_logger(), "SlalomAiFilter min_detection_before_success");
        _detection_number_for_average = getInput<AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_PARAM_TYPE>(AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_PARAM_NAME);

        // I put those two parameter to do the test of witch one we're gonna use.
        RCLCPP_INFO(_ros_node->get_logger(), "SlalomAiFilter max_frame_before_failing");
        _max_frame_before_failing = getInput<AI_FILTER_MAX_FRAME_BEFORE_FAILING_PARAM_TYPE>(AI_FILTER_MAX_FRAME_BEFORE_FAILING_PARAM_NAME);
        RCLCPP_INFO(_ros_node->get_logger(), "SlalomAiFilter max_frame_before_failing_sec");
        _max_time_before_failing = getInput<AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_PARAM_TYPE>(AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_PARAM_NAME);
        RCLCPP_INFO(_ros_node->get_logger(), "Stop stock input parameters");

    }

    void SlalomAiFilter::setting_output()
    {
        setOutput(SLALOM_AI_FILTER_RED_SLALOM_NAME, detection_average(_detection_array));
        setOutput(SLALOM_AI_FILTER_WHITE_SLALOM_NAME, detection_average(_detection_array_white_slalom));
    }

}  // namespace vision
