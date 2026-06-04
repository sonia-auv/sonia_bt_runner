#include "sonia_bt_runner/vision/TorpedoAiFilter.hpp"
#include "sonia_bt_runner/vision/ObjectVerification.hpp"

using std::placeholders::_1;
namespace vision {
    TorpedoAi::TorpedoAi(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), _ros_node(node), _detecting_target{false}, _timout_counter{}
    {
        
    }

    BT::NodeStatus TorpedoAi::onStart()
    {
        // We go get the information in the behavior tree
        _object = getInput<std::string>("Object_class");
        _confidence = getInput<float>("Confidence");
        _detection_number_for_average = getInput<int>("Min_detections_before_success");
        _two_objects_possible = getInput<int>("Two_objects_possible");

        // I put those two parameter to do the test of witch one we're gonna use.
        _max_frame_before_failing = getInput<int>("Max_frame_before_failing");
        _max_time_before_failing = getInput<float>("Max_time_before_failing_sec");

        _max_depth = getInput<float>("Max_depth");

        if (!verifyObject(_object.value()).has_value()) {
            RCLCPP_INFO(_ros_node->get_logger(), "The detected object is not a valid name of type of detection. Syntaxe error");
            return BT::NodeStatus::FAILURE;
        }

        if (_detection_number_for_average.value() <= 1)
        {
            RCLCPP_INFO(_ros_node->get_logger(), "You have to set the Min_detections_before_success parameter to more than 1.");
            return BT::NodeStatus::FAILURE;
        }

        // We initialize some value
        _launch_time = std::chrono::system_clock::now();

        // We initialize the subscriber to get the info of the detected image
        // if(_cam.value())
            _ai_filter_sub = _ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/front/classif", 1, std::bind(&AiFilter::ai_filter_callback, this, _1));
        // else
        //     _ai_filter_sub = _ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/bottom/classif", 1, std::bind(&AiFilter::ai_filter_callback, this, _1));
        
        // We run the node
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus TorpedoAi::onRunning(){

        std::chrono::duration<double> diff = std::chrono::system_clock::now() - _launch_time;
        _time_diff = diff.count();

        if (_detecting_target) {

        } else {

        }
        if((_max_frame_before_failing.value() != 0 && _timout_counter >= _max_frame_before_failing.value()) || (_max_time_before_failing.value() != 0.0 && _time_diff >= _max_time_before_failing.value()))
        {
            // We took to much time or count too many frame to fond the object
            if (_max_frame_before_failing.value() != 0)
                RCLCPP_INFO(_ros_node->get_logger(), "_timout_counter %d : max frame = %d, We don't find what we are looking for.", _timout_counter, _max_frame_before_failing.value());
            else
                RCLCPP_INFO(_ros_node->get_logger(), "node time %f : max time = %f, We don't find what we are looking for.", _time_diff, _max_time_before_failing.value());
            
            _ai_filter_sub.reset();
            return BT::NodeStatus::FAILURE;
        }

        if(_detection_array.size() < (size_t)_detection_number_for_average.value()){

            //No image or not enought image captured
            return BT::NodeStatus::RUNNING;
        }

        if (!_detecting_target) {
            _main_object
            _detecting_target = true;
        }

        // We kill the subscriber because we don't need more detection
        _ai_filter_sub.reset();

        // We need to make some selection in the image array
        RCLCPP_INFO(_ros_node->get_logger(), "Getting the information because enough detection have been made : %ld detection(s)", _detection_array.size());

        // We select the detection that we want to use to compute the output
        std::vector<size_t> choosen_index;

        if (_two_objects_possible.value()) {
            multiple_object_possible(choosen_index);
        } else {
            one_object_possible(choosen_index);
        }

        // We compute the new average of every parameter of the detection that we keep.
        AiDetection output_detection{};
        output_detection.classification = _object.value();
        for(size_t i : choosen_index)
        {
            output_detection.distance += _detection_array[i].distance;
            output_detection.confidence += _detection_array[i].confidence;
            output_detection.angle_teta += _detection_array[i].angle_teta;
            output_detection.angle_alpha += _detection_array[i].angle_alpha;
            output_detection.distance_teta += _detection_array[i].distance_teta;
            output_detection.distance_beta += _detection_array[i].distance_beta;
        }
        output_detection.distance /= choosen_index.size();
        output_detection.confidence /= choosen_index.size();
        output_detection.angle_teta /= choosen_index.size();
        output_detection.angle_alpha /= choosen_index.size();
        output_detection.distance_teta /= choosen_index.size();
        output_detection.distance_beta /= choosen_index.size();

        setOutput("Detected_object", output_detection);

        return BT::NodeStatus::SUCCESS;
    }

    void TorpedoAi::onHalted() {}

    void TorpedoAi::ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) {
       
        _timout_counter++;
        for (auto msg_obj: msg.detected_object){
            if(msg_obj.class_name.compare(_object.value()) == 0){

                // The searching object has been detected
                RCLCPP_INFO(_ros_node->get_logger(), "Class OK");
                RCLCPP_INFO(_ros_node->get_logger(), "Detection before filter %s : dist = %f | conf = %f", msg_obj.class_name.c_str(), msg_obj.distance, msg_obj.confidence);
                RCLCPP_INFO(_ros_node->get_logger(), "Comparing %s and %s = %d", msg_obj.class_name.c_str(), _object.value().c_str(), msg_obj.class_name.compare(_object.value()));
                
                if(msg_obj.confidence >= _confidence.value() && msg_obj.distance <= _max_depth.value())
                {
                    //The detected object respect the confidence and the depth. We can put it in the filter array
                    RCLCPP_INFO(_ros_node->get_logger(), "Confidence and depth OK, a new object has been detected");
                    _detection_array.push_back(msg_obj);
                }
            }
        }
    }

    void TorpedoAi::one_object_possible(std::vector<size_t>& indexs)
    {
        double teta_average{};
        double beta_average{};
        size_t highest_index{};
        double highest_error{};

        // We compute a teta and beta average of the detection
        for (size_t i{}; i < _detection_array.size(); i++)
        {
            teta_average += _detection_array[i].distance_teta;
            beta_average += _detection_array[i].distance_beta;
        }
        teta_average /= _detection_array.size();
        beta_average /= _detection_array.size();

        // We flush the higest and the lowest error between the average and detection value
        double teta_error;
        double beta_error;
        double error;
        for (size_t i{}; i < _detection_array.size(); i++)
        {
            teta_error = _detection_array[i].distance_teta - teta_average;
            beta_error = _detection_array[i].distance_beta - beta_average;
            error = teta_error * teta_error + beta_error * beta_error;
            if (error > highest_error)
            {
                highest_index = i;
                highest_error = error;
            }
        }
        for (size_t i{}; i < _detection_array.size(); i++)
        {
            if (i != highest_index)
            {
                indexs.push_back(i);
            }
        }
    }

    void TorpedoAi::multiple_object_possible(std::vector<size_t>& indexs)
    {
        double smallest_distance {_detection_array[0].distance_teta * _detection_array[0].distance_teta + _detection_array[0].distance_beta * _detection_array[0].distance_beta};
        size_t index_of_smallest_distance{};
        float i_centered_distance;
        
        // We compute the closest detection to the center of the camera
        for (size_t i{1}; i < _detection_array.size(); i++)
        {
            i_centered_distance = _detection_array[i].distance_teta * _detection_array[i].distance_teta + _detection_array[i].distance_beta * _detection_array[i].distance_beta;
            if (i_centered_distance < smallest_distance)
            {
                smallest_distance = i_centered_distance;
                index_of_smallest_distance = i;
            }
        }

        // We choose the detection on a distance of 10 cm with the closest one.
        indexs.push_back(index_of_smallest_distance);
        double beta;
        double teta;
        for (size_t i{};i < _detection_array.size(); i++)
        {
            if (i != index_of_smallest_distance)
            {
                beta = _detection_array[i].distance_beta - _detection_array[index_of_smallest_distance].distance_beta;
                teta = _detection_array[i].distance_teta - _detection_array[index_of_smallest_distance].distance_teta;
                if (0.1 >= sqrt(beta * beta + teta * teta))
                {
                    indexs.push_back(i);
                }
            }
        }
    }
}  // namespace vision