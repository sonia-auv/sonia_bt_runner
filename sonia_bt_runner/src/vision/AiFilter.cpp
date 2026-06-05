#include "sonia_bt_runner/vision/AiFilter.hpp"
#include "sonia_bt_runner/vision/ObjectVerification.hpp"
#include <cmath>

using std::placeholders::_1;
namespace vision{
    AiFilter::AiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), _ros_node(node), _detection_array()
    {
    }

    BT::NodeStatus AiFilter::onStart()
    {
        // We go get the information in the behavior tree
        _cam = getInput<int>("Camera");
        _object = getInput<std::string>("Object");
        _confidence = getInput<float>("Confidence");
        _max_depth = getInput<float>("Max_depth");
        _detection_number_for_average = getInput<int>("Detection_number_for_average");

        // I put those two parameter to do the test of witch one we're gonna use.
        _max_frame_before_failing = getInput<int>("Max_frame_number_before_failing");
        _max_time_before_failing = getInput<float>("Max_time_before_failing_sec");

        // We verify if the object is valid
        if (!verifyObject(_object.value()).has_value()) {
            RCLCPP_INFO(_ros_node->get_logger(), "The detected object is not a valid name of type of detection. Syntaxe error");
            return BT::NodeStatus::FAILURE;
        }

        if (_detection_number_for_average.value() <= 1)
        {
            RCLCPP_INFO(_ros_node->get_logger(), "You have to set the Min_detections_before_success parameter to more than 1.");
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus AiFilter::onRunning()
    {
        if(!detection_complete()) {
            return BT::NodeStatus::FAILURE;
        }

        // We need to make some selection in the image array
        RCLCPP_INFO(_ros_node->get_logger(), "Getting the information because enough detection have been made : %ld detection(s)", _detection_array.size());

        // We select the detection that we want to use to compute the output
        applicate_box_plot_to_detections();

        setOutput("Detected_object", detection_average());

        return BT::NodeStatus::SUCCESS;
    }

    void AiFilter::onHalted()
    {
    }

    bool AiFilter::is_object_found(const std::string& object, const float confidence, const float max_depth)
    {

        float time_diff;

        //We set the value for the detection
        _object_filter = object;
        _confidence_filter = confidence;
        _max_depth_filter = max_depth;
        _detection_array.clear();
        _timout_counter = 0;
        std::chrono::duration<double> diff;
        std::chrono::_V2::system_clock::time_point launch_time(std::chrono::system_clock::now());
        
        // We create the subscriber to gather the information
        if(_cam.value())
            _ai_filter_sub = _ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/front/classif", 1, std::bind(&AiFilter::ai_filter_callback, this, _1));
        else
            _ai_filter_sub = _ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/bottom/classif", 1, std::bind(&AiFilter::ai_filter_callback, this, _1));
        
        // We now wait for the detection
        while (
            (_max_frame_before_failing.value() == 0 || _timout_counter < _max_frame_before_failing.value())
            && (_max_time_before_failing.value() == 0.0f || time_diff < _max_time_before_failing.value())
            && _detection_array.size() < (size_t)_detection_number_for_average.value())
        {
            diff = std::chrono::system_clock::now() - launch_time;
            time_diff = diff.count();
        }
        
        // We reset the subscriber because we don't want to update the number of detection in our array
        _ai_filter_sub.reset();
        
        return _detection_array.size() >= (size_t)_detection_number_for_average.value();
    }

    void AiFilter::ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) {
       
        _timout_counter++;
        for (auto msg_obj: msg.detected_object){
            if(msg_obj.class_name.compare(_object_filter) == 0)
            {
                // The searching object has been detected
                // RCLCPP_INFO(_ros_node->get_logger(), "Class OK");
                // RCLCPP_INFO(_ros_node->get_logger(), "Detection before filter %s : dist = %f | conf = %f", msg_obj.class_name.c_str(), msg_obj.distance, msg_obj.confidence);
                // RCLCPP_INFO(_ros_node->get_logger(), "Comparing %s and %s = %d", msg_obj.class_name.c_str(), _object.value().c_str(), msg_obj.class_name.compare(_object.value()));
                if(msg_obj.confidence >= _confidence_filter && msg_obj.distance <= _max_depth_filter)
                {
                    //The detected object respect the confidence and the depth. We can put it in the filter array
                    RCLCPP_INFO(_ros_node->get_logger(), "Confidence and depth OK, a new object has been detected");
                    _detection_array.push_back(msg_obj);
                }
            }
        }

    }

    bool AiFilter::detection_complete() 
    {
        if(!is_object_found(_object.value(), _confidence.value(), _max_depth.value()))
        {
            // We took to much time or count too many frame to fond the object
            if (_max_frame_before_failing.value() != 0)
                RCLCPP_INFO(_ros_node->get_logger(), "max frame = %d, We don't find what we are looking for.", _max_frame_before_failing.value());
            else
                RCLCPP_INFO(_ros_node->get_logger(), "max time = %f, We don't find what we are looking for.", _max_time_before_failing.value());
            return false;
        }
        return true;
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
            q1 = _detection_array[(size_t)std::floor((float)_detection_array.size()/4.0f)].distance_teta;
            q3 = _detection_array[(size_t)std::floor((float)_detection_array.size()*3.0f/4.0f)].distance_teta;
        } else {
            q1 = (_detection_array[(size_t)(_detection_array.size()/4)].distance_teta + _detection_array[(size_t)(_detection_array.size()/4) + (size_t)1].distance_teta)/2.0f;
            q3 = (_detection_array[(size_t)(_detection_array.size()*3/4)].distance_teta + _detection_array[(size_t)(_detection_array.size()*3/4) + (size_t)1].distance_teta)/2.0f;
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

    AiDetection AiFilter::detection_average()
    {
        // We compute the new average of every parameter of the detection that we keep.
        AiDetection output_detection{};
        output_detection.classification = _object.value();
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