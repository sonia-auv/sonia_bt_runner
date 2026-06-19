#include "sonia_bt_runner/vision/AiFilter.hpp"
#include "sonia_bt_runner/vision/ObjectVerification.hpp"
#include <cmath>

using std::placeholders::_1;
namespace vision{
    AiFilter::AiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), _launch_time(), _ros_node(node), _detection_array(), _timout_counter{}
    {
    }

    BT::NodeStatus AiFilter::onStart()
    {
        RCLCPP_INFO(_ros_node->get_logger(), "AiFilter OnStart start");
        stock_input_parameters();

        if (!set_filter_parameter(_object.value(), _confidence.value(), _max_depth.value())) {
            return BT::NodeStatus::FAILURE;
        }

        if (!initial_condition_verification()) {
            return BT::NodeStatus::FAILURE;
        }

        initialize_subscriber();

        RCLCPP_INFO(_ros_node->get_logger(), "AiFilter OnStart finish");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus AiFilter::onRunning()
    {
        RCLCPP_INFO(_ros_node->get_logger(), "AiFilter OnRunning start");
        RCLCPP_INFO(_ros_node->get_logger(), "AiFilter get_detection_status start");
        BT::NodeStatus detection_status = get_detection_status();
        RCLCPP_INFO(_ros_node->get_logger(), "AiFilter get_detection_status stop");
        if(detection_status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(_ros_node->get_logger(), "AiFilter detection success start");

            
            RCLCPP_INFO(_ros_node->get_logger(), "AiFilter delete subscriber start");
            delete_subscriber();

            RCLCPP_INFO(_ros_node->get_logger(), "Getting the information because enough detection have been made : %ld detection(s)", _detection_array.size());

            applicate_box_plot_to_detections();
            RCLCPP_INFO(_ros_node->get_logger(), "AiFilter applicate_box_plot_to_detections stop");

            setting_output();
            RCLCPP_INFO(_ros_node->get_logger(), "AiFilter setting_output start");
        }

        return detection_status;
    }

    void AiFilter::onHalted()
    {
    }

    void AiFilter::stock_input_parameters()
    {
        // We go get the information in the behavior tree
        _cam = getInput<AI_FILTER_CAMERA_TYPE>(AI_FILTER_CAMERA_NAME);
        _object = getInput<AI_FILTER_OBJECT_CLASS_TYPE>(AI_FILTER_OBJECT_CLASS_NAME);
        _confidence = getInput<AI_FILTER_CONFIDENCE_TYPE>(AI_FILTER_CONFIDENCE_NAME);
        _max_depth = getInput<AI_FILTER_MAX_DEPTH_TYPE>(AI_FILTER_MAX_DEPTH_NAME);
        _detection_number_for_average = getInput<AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_TYPE>(AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_NAME);

        // I put those two parameter to do the test of witch one we're gonna use.
        _max_frame_before_failing = getInput<AI_FILTER_MAX_FRAME_BEFORE_FAILING_TYPE>(AI_FILTER_MAX_FRAME_BEFORE_FAILING_NAME);
        _max_time_before_failing = getInput<AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_TYPE>(AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_NAME);
    }

    bool AiFilter::set_filter_parameter(const std::string& object, const float confidence, const float max_depth)
    {
        if (!verifyObject(object)) {
            RCLCPP_INFO(_ros_node->get_logger(), "The detected object is not a valid name of type of detection. Syntaxe error");
            return false;
        }
        _object_filter = object;
        _confidence_filter = confidence;
        _max_depth_filter = max_depth;
        return true;
    }

    bool AiFilter::initial_condition_verification()
    {
        // We verify if the object is valid
        if (_detection_number_for_average.value() <= 1)
        {
            RCLCPP_WARN(_ros_node->get_logger(), "You have to set the Min_detections_before_success parameter to more than 1.");
            return false;
        }
        return true;
    }

    void AiFilter::initialize_subscriber()
    {
        if(_cam.value())
            _ai_filter_sub = _ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/front/classif", 1, std::bind(&AiFilter::ai_filter_callback, this, _1));
        else
            _ai_filter_sub = _ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/bottom/classif", 1, std::bind(&AiFilter::ai_filter_callback, this, _1));
        _launch_time = std::chrono::system_clock::now();
    }

    void AiFilter::delete_subscriber()
    {
        _ai_filter_sub.reset();
    }

    BT::NodeStatus AiFilter::get_detection_status()
    {
        std::chrono::duration<double> diff(std::chrono::system_clock::now() - _launch_time);
        float time_diff{(float)diff.count()};

        // We now wait for the detection
        if (_detection_array.size() >= (size_t)_detection_number_for_average.value())
        {
            RCLCPP_INFO(_ros_node->get_logger(), "Found enough detections: %ld", _detection_array.size());
            return BT::NodeStatus::SUCCESS;
        }
        else if (_max_time_before_failing.value() != 0.0f && time_diff >= _max_time_before_failing.value())
        {
            RCLCPP_INFO(_ros_node->get_logger(), "max time = %f, We don't find what we are looking for.", _max_time_before_failing.value());
            return BT::NodeStatus::FAILURE;
        }
        else if (_max_frame_before_failing.value() != 0 && _timout_counter >= _max_frame_before_failing.value())
        {
            RCLCPP_INFO(_ros_node->get_logger(), "max frame = %d, We don't find what we are looking for.", _max_frame_before_failing.value());
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void AiFilter::ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) {
       
        RCLCPP_INFO(_ros_node->get_logger(), "AiFilter callback running");
        _timout_counter++;
         RCLCPP_INFO(_ros_node->get_logger(), "Start ai_filter_callback");
        for (auto msg_obj: msg.detected_object){
         RCLCPP_INFO(_ros_node->get_logger(), "Received object: %s", msg_obj.class_name.c_str());
            if(msg_obj.class_name.compare(_object_filter) == 0)
            {
		 RCLCPP_INFO(_ros_node->get_logger(), "Get the wanted object: %s!!!", msg_obj.class_name.c_str());
                if(msg_obj.confidence >= _confidence_filter && msg_obj.distance <= _max_depth_filter)
                {
                    //The detected object respect the confidence and the depth. We can put it in the filter array
                    RCLCPP_INFO(_ros_node->get_logger(), "Confidence and depth OK, a new object has been detected");
                    _detection_array.push_back(msg_obj);
                }
            }
        }

    }

    void AiFilter::applicate_box_plot_to_detections()
    {
        float q1{};             // first quartile
        float q3{};             // third quartile
        float min{};            // min bound
        float max{};            // max bound

        // We made a filtering in ascending order of the teta distance
        sonia_common_ros2::msg::Detection temp;
        for (size_t i{}; i < _detection_array.size() - 1; ++i) {
            for (size_t j{}; j < _detection_array.size() - i - 1; ++j) {
                if (_detection_array[j].distance_teta > _detection_array[j + 1].distance_teta) {
                    temp = _detection_array[j];
                    _detection_array[j] = _detection_array[j + 1];
                    _detection_array[j + 1] = temp;
                }
            }
        }

        // We compute the quartile
        if (_detection_array.size() % 4) {
            q1 = _detection_array[std::floor((float)_detection_array.size()/4.0f)].distance_teta;
            q3 = _detection_array[std::floor((float)_detection_array.size()*3.0f/4.0f)].distance_teta;
        } else {
            q1 = (_detection_array[_detection_array.size()/4].distance_teta + _detection_array[_detection_array.size()/4 + 1].distance_teta)/2.0f;
            q3 = (_detection_array[_detection_array.size()*3/4].distance_teta + _detection_array[_detection_array.size()*3/4 + 1].distance_teta)/2.0f;
        }

        // We compute the min and max bound where the data will be kept
        min = q1 - 1.5f * (q3 - q1);
        max = q3 + 1.5f * (q3 - q1);

        // We keep the detection inside the min and the max bound
        for (std::vector<sonia_common_ros2::msg::Detection>::iterator it = _detection_array.begin(); it != _detection_array.end();) {
            if (it->distance_teta < min || it->distance_teta > max) {
                it = _detection_array.erase(it);
            } else {
                ++it;
            }
        }

        // We did the same thing with the distance_beta
        // We made a filtering in ascending order of the beta distance
        for (size_t i{}; i < _detection_array.size() - 1; ++i) {
            for (size_t j{}; j < _detection_array.size() - i - 1; ++j) {
                if (_detection_array[j].distance_beta > _detection_array[j + 1].distance_beta) {
                    temp = _detection_array[j];
                    _detection_array[j] = _detection_array[j + 1];
                    _detection_array[j + 1] = temp;
                }
            }
        }

        // We compute the quartile
        if (_detection_array.size() % 4) {
            q1 = _detection_array[(size_t)std::floor((float)_detection_array.size()/4.0f)].distance_beta;
            q3 = _detection_array[(size_t)std::floor((float)_detection_array.size()*3.0f/4.0f)].distance_beta;
        } else {
            q1 = (_detection_array[(size_t)(_detection_array.size()/4)].distance_beta + _detection_array[(size_t)(_detection_array.size()/4) + (size_t)1].distance_beta)/2.0f;
            q3 = (_detection_array[(size_t)(_detection_array.size()*3/4)].distance_beta + _detection_array[(size_t)(_detection_array.size()*3/4) + (size_t)1].distance_beta)/2.0f;
        }

        // We compute the min and max bound where the data will be kept
        min = q1 - 1.5f * (q3 - q1);
        max = q3 + 1.5f * (q3 - q1);

        // We keep the detection inside the min and the max bound
        for (std::vector<sonia_common_ros2::msg::Detection>::iterator it = _detection_array.begin(); it != _detection_array.end();) {
            if (it->distance_beta < min || it->distance_beta > max) {
                it = _detection_array.erase(it);
            } else {
                ++it;
            }
        }

        // Will propably do the same thing with the depth

    }

    AI_FILTER_DETECTED_OBJECT_TYPE AiFilter::detection_average(const std::vector<sonia_common_ros2::msg::Detection>& detection_array)
    {
        // We compute the new average of every parameter of the detection that we keep.
        AI_FILTER_DETECTED_OBJECT_TYPE output_detection{};
        output_detection.classification = _object.value();
        for(auto detection : detection_array)
        {
            output_detection.distance += detection.distance;
            output_detection.confidence += detection.confidence;
            output_detection.angle_teta += detection.angle_teta;
            output_detection.angle_alpha += detection.angle_alpha;
            output_detection.distance_teta += detection.distance_teta;
            output_detection.distance_beta += detection.distance_beta;
        }
        output_detection.distance /= detection_array.size();
        output_detection.confidence /= detection_array.size();
        output_detection.angle_teta /= detection_array.size();
        output_detection.angle_alpha /= detection_array.size();
        output_detection.distance_teta /= detection_array.size();
        output_detection.distance_beta /= detection_array.size();

        return output_detection;
    }

    void AiFilter::setting_output()
    {
        setOutput(AI_FILTER_DETECTED_OBJECT_NAME, detection_average(_detection_array));
    }

}  // namespace vision
