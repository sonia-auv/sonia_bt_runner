#include "sonia_bt_runner/vision/AiFilter.hpp"

using std::placeholders::_1;
namespace vision{
    AiFilter::AiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), ros_node(node)
    {
        
    }
    AiFilter::~AiFilter(){}
    BT::NodeStatus AiFilter::onStart(){

        // We go get the information in the behavior tree
        cam = getInput<int>("Camera");
        _object = getInput<std::string>("Object_class");
        confidence = getInput<float>("Confidence");
        min_detections_before_success = getInput<int>("Min_detections_before_success");
        two_objects_possible = getInput<int>("Two_objects_possible");

        // I put those two parameter to do the test of witch one we're gonna use.
        max_frame_before_failing = getInput<int>("Max_frame_before_failing");
        max_time_before_failing = getInput<float>("Max_time_before_failing_sec");

        max_depth = getInput<float>("Max_depth");

        if (min_detections_before_success.value() <= 1)
        {
            RCLCPP_INFO(ros_node->get_logger(), "You have to set the Min_detections_before_success parameter to more than 1.");
            return BT::NodeStatus::FAILURE;
        }

        // We initialize some value
        counter = 0;
        _launch_time = std::chrono::system_clock::now();

        // We chose the right camera to capture the image
        if(cam.value())
            ai_filter_sub = ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/front/classif", 1, std::bind(&AiFilter::ai_filter_callback, this, _1));
        else
            ai_filter_sub = ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/bottom/classif", 1, std::bind(&AiFilter::ai_filter_callback, this, _1));    
        
        // We run the node
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus AiFilter::onRunning(){

        std::chrono::duration<double> diff = std::chrono::system_clock::now() - _launch_time;
        time_diff = diff.count();

        if(max_frame_before_failing.value() != 0 && counter >= max_frame_before_failing.value() || max_time_before_failing.value() != 0.0 && time_diff >= max_time_before_failing.value())
        {
            // We took to much time or count too many frame to fond the object
            if (max_frame_before_failing.value() != 0)
                RCLCPP_INFO(ros_node->get_logger(), "counter %d : max frame = %d, We don't find what we are looking for.", counter, max_frame_before_failing.value());
            else
                RCLCPP_INFO(ros_node->get_logger(), "node time %f : max time = %f, We don't find what we are looking for.", time_diff, max_time_before_failing.value());
            
            ai_filter_sub.reset();
            return BT::NodeStatus::FAILURE;
        }

        if(_detection_array.size() < min_detections_before_success.value()){

            //No image or not enought image captured
            return BT::NodeStatus::RUNNING;
        }

        // We kill the subscriber because we don't need more detection
        ai_filter_sub.reset();

        // We need to make some selection in the image array
        RCLCPP_INFO(ros_node->get_logger(), "Getting the information because enough detection have been made : %ld detection(s)", _detection_array.size());

        // We compute the detection that we use to compute the output
        std::vector<int> choosen_index;

        if (two_objects_possible.value())
        {
            float smallest_distance = pow(_detection_array[0].distance_teta, 2) + pow(_detection_array[0].distance_beta, 2);
            float index_of_smallest_distance = 0;
            
            // We compute the closest detection to the center of the camera
            for (int i = 1; i < _detection_array.size(); i++)
            {
                float centered_distance = pow(_detection_array[i].distance_teta, 2) + pow(_detection_array[i].distance_beta, 2);
                if (centered_distance < smallest_distance)
                {
                    smallest_distance = centered_distance;
                    index_of_smallest_distance = i;
                }
            }

            // We choose the detection on a distance of 10 cm with the closest one.
            choosen_index.push_back(index_of_smallest_distance);
            for (int i = 0;i < _detection_array.size(); i++)
            {
                if (i != index_of_smallest_distance)
                {
                    if (0.1 >= sqrt(pow(_detection_array[i].distance_beta - _detection_array[index_of_smallest_distance].distance_beta, 2) + pow(_detection_array[i].distance_teta - _detection_array[index_of_smallest_distance].distance_teta, 2)))
                    {
                        choosen_index.push_back(i);
                    }
                }
            }
        }
        else
        {
            float teta_average;
            float beta_average;
            int highest_index = 0;
            float highest_error = 0.0;

            // We compute a teta and beta average of the detection
            for (int i = 0; i < _detection_array.size(); i++)
            {
                teta_average += _detection_array[i].distance_teta/_detection_array.size();
                beta_average += _detection_array[i].distance_beta/_detection_array.size();
            }

            // We flush the higest and the lowest error between the average and detection value
            for (int i = 0; i < _detection_array.size(); i++)
            {
                float teta_error = _detection_array[i].distance_teta - teta_average;
                float beta_error = _detection_array[i].distance_beta - beta_average;
                float error = pow(teta_error,2) + pow(beta_error,2);
                if (error > highest_error)
                {
                    highest_index = i;
                    highest_error = error;
                }
            }
            for (int i = 0; i < _detection_array.size(); i++)
            {
                if (i != highest_index)
                {
                    choosen_index.push_back(i);
                }
            }
        }

        // We compute the new average of every parameter of the detection that we keep.
        AiDetection output_detection;
        output_detection.classification = _object.value();
        output_detection.distance = 0.0;
        output_detection.confidence = 0.0;
        output_detection.angle_teta = 0.0;
        output_detection.angle_alpha = 0.0;
        output_detection.distance_teta = 0.0;
        output_detection.distance_beta = 0.0;
        for(int i : choosen_index)
        {
            output_detection.distance += _detection_array[i].distance/choosen_index.size();
            output_detection.confidence += _detection_array[i].confidence/choosen_index.size();
            output_detection.angle_teta += _detection_array[i].angle_teta/choosen_index.size();
            output_detection.angle_alpha += _detection_array[i].angle_alpha/choosen_index.size();
            output_detection.distance_teta += _detection_array[i].distance_teta/choosen_index.size();
            output_detection.distance_beta += _detection_array[i].distance_beta/choosen_index.size();
        }

        setOutput("Detected_object", output_detection);

        return BT::NodeStatus::SUCCESS;
    }
    void AiFilter::onHalted()
    {
    }

    void AiFilter::ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) {
       
        counter++;
        for (auto msg_obj: msg.detected_object){
            
            if(msg_obj.class_name.compare(_object.value()) == 0){

                // The searching object has been detected
                RCLCPP_INFO(ros_node->get_logger(), "Class OK");
                RCLCPP_INFO(ros_node->get_logger(), "Detection before filter %s : dist = %f | conf = %f", msg_obj.class_name.c_str(), msg_obj.distance, msg_obj.confidence);
                RCLCPP_INFO(ros_node->get_logger(), "Comparing %s and %s = %d", msg_obj.class_name.c_str(), _object.value().c_str(), msg_obj.class_name.compare(_object.value()));
                
                if(msg_obj.confidence >= confidence.value() && msg_obj.distance <= max_depth.value())
                {
                    //The detected object respect the confidence and the depth. We can put it in the filter array
                    RCLCPP_INFO(ros_node->get_logger(), "Confidence and depth OK, a new object has been detected");
                    _detection_array.push_back(msg_obj);
                }
            }
        }
    }
}  // namespace vision