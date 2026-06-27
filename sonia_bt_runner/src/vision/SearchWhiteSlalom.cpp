#include "sonia_bt_runner/vision/SearchWhiteSlalom.hpp"
#include "sonia_bt_runner/utils/NormalizeDetection.hpp"

using std::placeholders::_1;

namespace vision {

SearchWhiteSlalom::SearchWhiteSlalom(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), _ros_node(node)
{}

BT::NodeStatus SearchWhiteSlalom::onStart()
{
    _red_detection = getInput<AiDetection>("RedSlalomDetection");
    _side = getInput<std::string>("Side");
    _confidence = getInput<float>("Confidence");
    _min_detections = getInput<int>("Min_detections_before_success");
    _max_time = getInput<float>("Max_time_before_failing_sec");
    _max_depth = getInput<float>("Max_depth");

    if (!_red_detection || !_side || !_confidence || !_min_detections || !_max_time || !_max_depth) {
        RCLCPP_ERROR(_ros_node->get_logger(), "SearchWhiteSlalom: missing required input port");
        return BT::NodeStatus::FAILURE;
    }

    if (_side.value() != "Left" && _side.value() != "Right") {
        RCLCPP_ERROR(_ros_node->get_logger(), "SearchWhiteSlalom: Side must be 'Left' or 'Right', got '%s'", _side.value().c_str());
        return BT::NodeStatus::FAILURE;
    }

    _valid_detections.clear();
    _time_diff = 0.0f;
    _launch_time = std::chrono::system_clock::now();

    _sub = _ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>(
        "/proc_vision/front/classif", 1,
        std::bind(&SearchWhiteSlalom::detection_callback, this, _1));

    RCLCPP_INFO(_ros_node->get_logger(), "SearchWhiteSlalom: searching for white slalom on the %s side of red slalom (angle_teta=%.2f)",
                _side.value().c_str(), _red_detection.value().angle_teta);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchWhiteSlalom::onRunning()
{
    std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - _launch_time;
    _time_diff = static_cast<float>(elapsed.count());

    if (_max_time.value() != 0.0f && _time_diff >= _max_time.value()) {
        RCLCPP_WARN(_ros_node->get_logger(), "SearchWhiteSlalom: timed out after %.2fs with only %zu valid detections",
                    _time_diff, _valid_detections.size());
        _sub.reset();
        return BT::NodeStatus::FAILURE;
    }

    if (static_cast<int>(_valid_detections.size()) < _min_detections.value()) {
        return BT::NodeStatus::RUNNING;
    }

    _sub.reset();

    AiDetection output{};
    output.classification = "WHITE_SLALOM";
    for (const auto &d : _valid_detections) {
        output.distance       += d.distance;
        output.confidence     += d.confidence;
        output.angle_teta     += d.angle_teta;
        output.angle_alpha    += d.angle_alpha;
        output.distance_teta  += d.distance_teta;
        output.distance_beta  += d.distance_beta;
    }
    float n = static_cast<float>(_valid_detections.size());
    output.distance      /= n;
    output.confidence    /= n;
    output.angle_teta    /= n;
    output.angle_alpha   /= n;
    output.distance_teta /= n;
    output.distance_beta /= n;

    setOutput("WhiteSlalomDetection", output);

    RCLCPP_INFO(_ros_node->get_logger(), "SearchWhiteSlalom: found white slalom at angle_teta=%.2f after %zu detections",
                output.angle_teta, _valid_detections.size());

    return BT::NodeStatus::SUCCESS;
}

void SearchWhiteSlalom::onHalted()
{
    _sub.reset();
}

void SearchWhiteSlalom::detection_callback(const sonia_common_ros2::msg::DetectionArray &msg)
{
    float red_angle = _red_detection.value().angle_alpha;
    bool want_left = (_side.value() == "Left");

    for (auto obj : msg.detected_object) {
        if (obj.class_name != "WHITE_SLALOM") continue;
        if (obj.confidence < _confidence.value()) continue;
        if (obj.distance > _max_depth.value()) continue;

        // Accept the detection only if it is on the requested side of the red slalom.
        // angle_teta < 0 means left of center, > 0 means right of center.
        bool is_left_of_red = (obj.angle_alpha > red_angle);
        if (want_left != is_left_of_red) continue;

	utils::normalize_detection(obj);
        _valid_detections.push_back(obj);
        RCLCPP_INFO(_ros_node->get_logger(), "SearchWhiteSlalom: valid detection #%zu — angle_teta=%.2f conf=%.2f dist=%.2f",
                    _valid_detections.size(), obj.angle_teta, obj.confidence, obj.distance);
    }
}

}  // namespace vision
