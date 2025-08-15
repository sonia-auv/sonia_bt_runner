#include "sonia_bt_runner/vision/AiFilter.hpp"

using std::placeholders::_1;
namespace vision{
    AiFilter::AiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), ros_node(node)
    {
        
    }
    AiFilter::~AiFilter(){}
    BT::NodeStatus AiFilter::onStart(){
        _object = getInput<std::string>("Object_class");
        max_frame= getInput<int>("Max_frame");
        max_size_output = getInput<int>("Max_size_output");
        confidence = getInput<float>("Confidence");
        max_depth = getInput<float>("Max_depth");
        min_detection = getInput<int>("Min_detection");

        BT::Expected<int> cam = getInput<int>("Camera");
        counter = 0;
        nb_detection = 0;
        //chose camera
        if(cam.value())
            ai_filter_sub = ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/front/classif", 1, std::bind(&AiFilter::ai_filter_callback, this, _1));
        else
            ai_filter_sub = ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/bottom/classif", 1, std::bind(&AiFilter::ai_filter_callback, this, _1));    
        
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus AiFilter::onRunning(){
        
        if(counter >= max_frame.value()){
            RCLCPP_INFO(ros_node->get_logger(), "counter %d : max frame = %d", counter, max_frame.value());
            return BT::NodeStatus::FAILURE;
        }


        if(_detection_array.empty()){
            return BT::NodeStatus::RUNNING;
        }

        AiDetectionArray detected_object_array;

        for (long unsigned int i = 0; i < _detection_array.size(); i++){
            AiDetection detected_object;
            detected_object.top_right_x = _detection_array[i].top_right_x;
            detected_object.top_right_y = _detection_array[i].top_right_y;
            detected_object.top_left_x = _detection_array[i].top_left_x;
            detected_object.top_left_y = _detection_array[i].top_left_y;
            
            detected_object.bottom_right_x = _detection_array[i].bottom_right_x;
            detected_object.bottom_right_y = _detection_array[i].bottom_right_y;
            detected_object.bottom_left_x = _detection_array[i].bottom_left_x;
            detected_object.bottom_left_y = _detection_array[i].bottom_left_y;

            detected_object.distance = _detection_array[i].distance;
            detected_object.confidence = _detection_array[i].confidence;
            detected_object.classification = _detection_array[i].class_name;

            RCLCPP_INFO(ros_node->get_logger(), "Detection filtered %s : dist = %f | conf = %f", detected_object.classification.c_str(), detected_object.distance, detected_object.confidence);


            detected_object_array.detection_array.push_back(detected_object);
        } 

        if(detected_object_array.detection_array.size() > max_size_output.value()){
            std::vector<float> distances;

            std::vector<int> ids;
            for (int n = 0; n < max_size_output.value(); n++){
                distances.push_back(100000);
                ids.push_back(0);
            }


            int i = 0;
            for (AiDetection detected_object: detected_object_array.detection_array){
                int k = 0;
                float max_dist = 0;
                int max_id = 0;
                for (float d: distances){
                    if (d > max_dist){
                        max_dist = d;
                        max_id = k;
                    }
                    k++;
                }

                if (max_dist > detected_object.distance){
                    ids[max_id] = i;
                }
                i++;
            }

            AiDetectionArray reduced_detected_object_array;
            int u = 0;

            for (AiDetection detected_object: detected_object_array.detection_array){
                float center_x =detected_object.top_left_x+detected_object.bottom_right_x;
                RCLCPP_INFO(ros_node->get_logger(), "Output : (IN FOR) class %s : center on x = %f | dist = %f", detected_object.classification.c_str(), center_x, detected_object.distance);

            }

            for (int index: ids){
                RCLCPP_INFO(ros_node->get_logger(), "Reducing id = %d | dist = %f", index, distances[u]);
                u++;
                reduced_detected_object_array.detection_array.push_back(detected_object_array.detection_array[index]);
            }
            RCLCPP_INFO(ros_node->get_logger(), "Output is reduced");
            float center_x =reduced_detected_object_array.detection_array[0].top_left_x+reduced_detected_object_array.detection_array[0].bottom_right_x;
            RCLCPP_INFO(ros_node->get_logger(), "Output : (AFTER REDUCING) class %s : center on x = %f | dist = %f", reduced_detected_object_array.detection_array[0].classification.c_str(), center_x, reduced_detected_object_array.detection_array[0].distance);

            setOutput("detected_object_array", reduced_detected_object_array);
        }
        else{
            RCLCPP_INFO(ros_node->get_logger(), "Output is not reduced");
            float center_x =detected_object_array.detection_array[0].top_left_x+detected_object_array.detection_array[0].bottom_right_x;
            RCLCPP_INFO(ros_node->get_logger(), "Output : (NOT REDUCING)class %s : center on x = %f | dist = %f", detected_object_array.detection_array[0].classification.c_str(), center_x, detected_object_array.detection_array[0].distance);
            setOutput("detected_object_array", detected_object_array);
        }

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

            if(msg_obj.class_name.compare(_object.value()) == 0){
                RCLCPP_INFO(ros_node->get_logger(), "Class OK");

                if(msg_obj.confidence >= confidence.value() && msg_obj.distance <= max_depth.value())
                {
                    RCLCPP_INFO(ros_node->get_logger(), "Confidence and depth OK");
                    nb_detection++;
                    if(nb_detection < min_detection.value()){
                        // nb_detection++;
                        RCLCPP_INFO(ros_node->get_logger(), "Detection on frame %d, nb_detection = %d", counter, nb_detection);
                        continue;
                    }
                    else{
                        _detection_array.push_back(msg_obj);
                    }
                }
            }
        }     
    }
}  // namespace vision