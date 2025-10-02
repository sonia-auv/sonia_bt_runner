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
        max_frame_before_failling= getInput<int>("Max_frame_before_failling");
        max_size_output = getInput<int>("Max_size_output");
        confidence = getInput<float>("Confidence");
        max_depth = getInput<float>("Max_depth");
        min_size_output = getInput<int>("Min_size_output");

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
        
        if(counter >= max_frame_before_failling.value()){

            // We took to much time to find the object
            RCLCPP_INFO(ros_node->get_logger(), "counter %d : max frame = %d, We don't find what we are looking for.", counter, max_frame_before_failling.value());
            return BT::NodeStatus::FAILURE;
        }


        if(_detection_array.size() < min_size_output.value()){

            //No image or not enought image captured
            return BT::NodeStatus::RUNNING;
        }
        // We need to make some selection in the image array
        // std::vector<float> distances;

        RCLCPP_INFO(ros_node->get_logger(), "Getting the information because enough detection have been made : %d detection(s)", _detection_array.size());
        std::vector<int> ids;
        for (int i = 0; i < max_size_output.value(); i++){
            ids.push_back(i);
        } 

        // We sort all the id distance in the ids vector
        if (_detection_array.size() > max_size_output.value())
        {
            RCLCPP_INFO(ros_node->get_logger(), "Detection number to High : %d detection(s)", _detection_array.size());
            for (int i = 0;i < max_size_output.value();i++)
            {
                float min_distance_found = 65.0;
                int min_distance_index = -1;
                int temp_id = 0;
                for(int j = i;j < max_size_output.value();j++)
                {
                    if (min_distance_found > _detection_array[j].distance)
                    {
                        min_distance_found = _detection_array[j].distance;
                        min_distance_index = j;
                    }
                }
                if (min_distance_index != -1)
                {
                    temp_id = ids[i];
                    ids[i] = ids[min_distance_index];
                    ids[min_distance_index] = temp_id;
                }
                
            }

            // We put the object with a smaller distance in the ids vector
            for (int i = max_size_output.value();i < _detection_array.size();i++)
            {
                if(_detection_array[i].distance < _detection_array[ids.back()].distance)
                {

                    // The object has a distance lower 
                    int j = max_size_output.value() - 1;
                    do
                    {
                        if (j == 0)
                        {
                            break;
                        }
                        j--;
                    }while(_detection_array[ids[j - 1]].distance < _detection_array[i].distance);
                    int temp1 = ids[j];
                    int temp2;
                    ids[j] = i;
                    for(int k = j;k < max_size_output.value();k++)
                    {
                        if(k == max_size_output.value() - 1)
                        {
                            ids[k] = temp1;
                        }
                        else
                        {
                            temp2 = ids[k];
                            ids[k] = temp1;
                            temp1 = temp2;
                        }
                    }
                }
            }
        }
        

        AiDetectionArray reduced_detected_object_array;

        for (int index: ids){

            // We fill the detected object
            RCLCPP_INFO(ros_node->get_logger(), "Index retains : %d", index);
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
            detected_object.angle_alpha = _detection_array[index].angle_alpha;
            detected_object.angle_teta = _detection_array[index].angle_teta;
            detected_object.distance_teta = _detection_array[index].distance_teta;
            detected_object.confidence = _detection_array[index].confidence;
            detected_object.classification = _detection_array[index].class_name;

            RCLCPP_INFO(ros_node->get_logger(), "Detection filtered %s : dist = %f | conf = %f", detected_object.classification.c_str(), detected_object.distance, detected_object.confidence);

            // We put the detected object in the detected array
            reduced_detected_object_array.detection_array.push_back(detected_object);
            RCLCPP_INFO(ros_node->get_logger(), "Reducing id = %d | dist = %f", index, _detection_array[index].distance);
        }
        setOutput("Detected_object_array", reduced_detected_object_array);

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

                if(msg_obj.confidence.value() >= confidence.value() && msg_obj.distance.value() <= max_depth.value())
                {

                    //The detected object respect the confidence and the depth enter in the behavior tree
                    RCLCPP_INFO(ros_node->get_logger(), "Confidence and depth OK, a new object has been detected");
                    _detection_array.push_back(msg_obj);
                }
            }
        }
    }
}  // namespace vision