#include "sonia_bt_runner/vision/SlalomAiFilter.hpp"

using std::placeholders::_1;

namespace vision {

    SlalomAiFilter::SlalomAiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : AiFilter(name, config, node)
{}


    void SlalomAiFilter::ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg)
    {

        std::optionnal<sonia_common_ros2::msg::Detection> closest_red_slalom;
        std::vector<sonia_common_ros2::msg::Detection> whites_slaloms;
        sonia_common_ros2::msg::Detection choosen_white_slalom;

        for (auto msg_obj : msg) {
            // We first detect the closest red slalom on the image
            if (msg_obj.class_name.compare("RED_SLALOM") == 0
             && msg_obj.confidence >= _confidence_filter
             && msg_obj.distance <= _max_depth_filter
             && (!closest_red_slalom || closest_red_slalom.distance >= msg_obj.distance))
            {
                closest_red_slalom = msg_obj;
            }

            // We detect the whites slaloms
            if (msg_obj.class_name.compare("WHITE_SLALOM") == 0
                && msg_obj.confidence >= _confidence_filter
                && msg_obj.distance <= _max_depth_filter)
            {
                white_slaloms.push_back(msg_obj);
            }
        }
        
        // We verify if we have both a RED_SLALOM and at least one WHITE_SLALOM
        if (!closest_red_slalom.has_value() || !whites_slaloms.size()) {
            return;
        }

        // We push_back the red and white slalom
        _detetection_array.push_back(closest_red_slalom.value());

        // We verify the depth on the detection

        // float red_angle = _red_detection.value().angle_teta;
        // bool want_left = (_side.value() == "Left");

        // for (const auto &obj : msg.detected_object) {
        //     if (obj.class_name != "WHITE_SLALOM") continue;
        //     if (obj.confidence < _confidence.value()) continue;
        //     if (obj.distance > _max_depth.value()) continue;

        //     // Accept the detection only if it is on the requested side of the red slalom.
        //     // angle_teta < 0 means left of center, > 0 means right of center.
        //     bool is_left_of_red = (obj.angle_teta < red_angle);
        //     if (want_left != is_left_of_red) continue;

        //     _valid_detections.push_back(obj);
        //     RCLCPP_INFO(_ros_node->get_logger(), "SearchWhiteSlalom: valid detection #%zu — angle_teta=%.2f conf=%.2f dist=%.2f",
        //                 _valid_detections.size(), obj.angle_teta, obj.confidence, obj.distance);

    }

    void SlalomAiFilter::parameter_setter()
    {
        // We go get the information in the behavior tree
        _side = getInput<std::stirng>("Side");
        _cam = getInput<int>("Camera");
        _confidence = getInput<float>("Confidence");
        _max_depth = getInput<float>("Max_depth");
        _detection_number_for_average = getInput<int>("Min_detections_before_success");

        // I put those two parameter to do the test of witch one we're gonna use.
        _max_frame_before_failing = getInput<int>("Max_frame_number_before_failing");
        _max_time_before_failing = getInput<float>("Max_time_before_failing_sec");
    }

    bool SlalomAiFilter::condition_verification()
    {
        if (_detection_number_for_average.value() <= 1)
        {
            RCLCPP_INFO(_ros_node->get_logger(), "You have to set the Min_detections_before_success parameter to more than 1.");
            return false;
        }
        return true;
    }

    void SlalomAiFilter::setting_output()
    {
        setOutput("Red_slalom", AiFilter::detection_average(_detection_array));
        setOutput("White_slalom", AiFilter::detection_average(_detection_array_white_slalom));
    }

}  // namespace vision
