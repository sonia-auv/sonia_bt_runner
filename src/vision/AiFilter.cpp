#include "sonia_bt_runner/vision/AiFilter.hpp"
#include <algorithm>

using std::placeholders::_1;
namespace vision{
    AiFilter::AiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), ros_node(node)
    {
        
    }
    AiFilter::~AiFilter(){}
    BT::NodeStatus AiFilter::onStart(){

        // We go get the information in the behavior tree
        _object = getInput<std::string>("Object_class");
        max_frame_before_exiting= getInput<int>("Max_frame_before_exiting");
        max_size_output = getInput<int>("Max_size_output");
        confidence = getInput<float>("Confidence");
        max_depth = getInput<float>("Max_depth");
        min_detection = getInput<int>("Min_detection");

        BT::Expected<int> cam = getInput<int>("Camera");

        // We initialize some value
        counter = 0;
        // nb_detection = 0;

        // We chose the right camera to capture the image
        if(cam.value())
            ai_filter_sub = ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/front/classif", 1, std::bind(&AiFilter::ai_filter_callback, this, _1));
        else
            ai_filter_sub = ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/bottom/classif", 1, std::bind(&AiFilter::ai_filter_callback, this, _1));    
        
        // We run the node
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus AiFilter::onRunning(){
        
        if(counter >= max_frame_before_exiting.value()){

            // We took to much time to find the object
            RCLCPP_INFO(ros_node->get_logger(), "counter %d : max frame = %d", counter, max_frame_before_exiting.value());
            return BT::NodeStatus::FAILURE;
        }


        if(_detection_array.size() < max_size_output.value()){

            //No image or not enought image captured
            return BT::NodeStatus::RUNNING;
        }
        // We need to make some selection in the image array
        // std::vector<float> distances;

        std::vector<int> ids;
        for (int n = 0; n < max_size_output.value(); n++){
            ids.push_back(-1);
        } 

        //We put a number on every detection related to the depth of them (>0 = far | 0 = closer)
        for (int i = 0; i < max_size_output.value(); i++)
        {
            int min_index_pointer = -1;
            float min_dist_found = 0.0;
            bool object_already_choosen = false;

            // We go get the closest object of the detected array not already choosen 
            for (int j = 0; j < _detection_array.size();j++)
            {
                if(min_index_pointer == -1){
                    min_index_pointer = j;
                    min_dist_found = _detection_array[j].distance;
                }
                else if (min_dist_found > _detection_array[j].distance && !item_is_in_vector(ids, j))
                {   
                    min_index_pointer = j;
                    min_dist_found = _detection_array[j].distance;
                }
            }
            ids[i] = min_index_pointer;
        }

        AiDetectionArray reduced_detected_object_array;

        for (int index: ids){

            // We fill the detected object 
            AiDetection detected_object;
            detected_object.top_right_x = _detection_array[index].top_right_x;
            detected_object.top_right_y = _detection_array[index].top_right_y;
            detected_object.top_left_x = _detection_array[index].top_left_x;
            detected_object.top_left_y = _detection_array[index].top_left_y;
            
            detected_object.bottom_right_x = _detection_array[index].bottom_right_x;
            detected_object.bottom_right_y = _detection_array[index].bottom_right_y;
            detected_object.bottom_left_x = _detection_array[index].bottom_left_x;
            detected_object.bottom_left_y = _detection_array[index].bottom_left_y;

            detected_object.distance = _detection_array[index].distance;
            detected_object.confidence = _detection_array[index].confidence;
            detected_object.classification = _detection_array[index].class_name;

            RCLCPP_INFO(ros_node->get_logger(), "Detection filtered %s : dist = %f | conf = %f", detected_object.classification.c_str(), detected_object.distance, detected_object.confidence);

            // We put the detected object in the detected array
            reduced_detected_object_array.detection_array.push_back(detected_object);
            RCLCPP_INFO(ros_node->get_logger(), "Reducing id = %d | dist = %f", index, detected_object_array.detection_array[index].distance);
        }

        RCLCPP_INFO(ros_node->get_logger(), "Output is reduced");
        float center_x =reduced_detected_object_array.detection_array[0].top_left_x+reduced_detected_object_array.detection_array[0].bottom_right_x;
        RCLCPP_INFO(ros_node->get_logger(), "Output : (AFTER REDUCING) class %s : center on x = %f | dist = %f", reduced_detected_object_array.detection_array[0].classification.c_str(), center_x, reduced_detected_object_array.detection_array[0].distance);

        setOutput("detected_object_array", reduced_detected_object_array);

        return BT::NodeStatus::SUCCESS;
    }
    void AiFilter::onHalted()
    {
    }

    void AiFilter::ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) {
       
        counter++;
        for (auto msg_obj: msg.detected_object){
            // TODO Tester la condition, devrait etre vraie si le nom de la classe est inclus dans le vecteur de noms de classes
            RCLCPP_INFO(ros_node->get_logger(), "Detection before filter %s : dist = %f | conf = %f", msg_obj.class_name.c_str(), msg_obj.distance, msg_obj.confidence);

            // if(std::find(_object.value().begin(), _object.value().end(), msg_obj.class_name) != _object.value().end()){
            RCLCPP_INFO(ros_node->get_logger(), "Comparing %s and %s = %d", msg_obj.class_name.c_str(), _object.value().c_str(), msg_obj.class_name.compare(_object.value()));

            if(msg_obj.class_name.compare(_object.value())){

                // The searching object has been detected
                RCLCPP_INFO(ros_node->get_logger(), "Class OK");

                if(msg_obj.confidence >= confidence.value() && msg_obj.distance <= max_depth.value())
                {

                    //The detected object respect the confidence and the depth enter in the behavior tree
                    RCLCPP_INFO(ros_node->get_logger(), "Confidence and depth OK, a new object has been detected");
                    _detection_array.push_back(msg_obj);
                }
            }
        }     
    }
}  // namespace vision