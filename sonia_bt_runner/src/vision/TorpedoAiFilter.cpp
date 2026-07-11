#include "sonia_bt_runner/vision/TorpedoAiFilter.hpp"
#include "sonia_bt_runner/utils/NormalizeDetection.hpp"

namespace vision {
    TorpedoAiFilter::TorpedoAiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :AiFilter(name, config, node)
    {
        
    }

    void TorpedoAiFilter::ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) 
    {
	AbstractAiFilter::ai_filter_callback(msg);

        sonia_common_ros2::msg::Detection object;
        std::vector<sonia_common_ros2::msg::Detection> target;
        bool detection_flag{false};
        size_t detection_id{};
        float smallest_distance{};
        float beta{};
        float teta{};

        for (auto msg_obj: msg.detected_object){
            utils::normalize_detection(msg_obj);
            if(msg_obj.class_name.compare(_object_class) == 0)
            {
                // The searching object has been detected
                RCLCPP_INFO(get_logger(), "Class OK");
                RCLCPP_INFO(get_logger(), "Detection before filter %s : dist = %f | conf = %f", msg_obj.class_name.c_str(), msg_obj.distance, msg_obj.confidence);
                RCLCPP_INFO(get_logger(), "Comparing %s and %s = %d", msg_obj.class_name.c_str(), _object_class.c_str(), msg_obj.class_name.compare(_object_class));
                if(msg_obj.confidence >= confidence() && msg_obj.distance <= max_depth())
                {
                    //The detected object respect the confidence and the depth. We can put it in the filter array
                    RCLCPP_INFO(get_logger(), "Confidence and depth OK, a new object has been detected");
                    object = msg_obj;
                    detection_flag = true;
                    break;
                }
            }
        }
        if (detection_flag) {
            for (auto msg_obj: msg.detected_object)
            {
                if(msg_obj.class_name.compare("CIRCLE") == 0)
                {
                    // The searching object has been detected
                    RCLCPP_INFO(get_logger(), "Class OK");
                    RCLCPP_INFO(get_logger(), "Detection before filter %s : dist = %f | conf = %f", msg_obj.class_name.c_str(), msg_obj.distance, msg_obj.confidence);
                    RCLCPP_INFO(get_logger(), "Comparing %s and %s = %d", msg_obj.class_name.c_str(), "CIRCLE", msg_obj.class_name.compare("CIRCLE"));
                    if(msg_obj.confidence >= confidence() && msg_obj.distance <= max_depth())
                    {
                        //The detected object respect the confidence and the depth. We can put it in the filter array
                        RCLCPP_INFO(get_logger(), "Confidence and depth OK, a new object has been detected");
                        target.push_back(msg_obj);
                    }
                }
            }
            if (target.size() > 0) {
                teta = (target[detection_id].distance_teta - object.distance_teta);
                beta = (target[detection_id].distance_beta - object.distance_beta);
                smallest_distance = (teta * teta + beta * beta);
                for (size_t i{1}; i < target.size();++i) {
                    teta = (target[i].distance_teta - object.distance_teta);
                    beta = (target[i].distance_beta - object.distance_beta);
                    if (smallest_distance > (teta * teta + beta * beta)) {
                        smallest_distance = teta * teta + beta * beta;
                        detection_id = i;
                    }
                }
                _detection_array.push_back(target[detection_id]);
            }
        }
        
    }
}  // namespace vision
