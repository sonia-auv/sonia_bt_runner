#include "sonia_bt_runner/vision/AiActivation.hpp"

namespace vision{
    AiActivation::AiActivation(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::SyncActionNode(name, config), _ros_node(node){
        _ai_client = _ros_node->create_client<sonia_common_ros2::srv::AiActivationService>("/proc_vision/ai_activation");
    }

    BT::NodeStatus AiActivation::tick(){
        std::shared_ptr<sonia_common_ros2::srv::AiActivationService_Response> response;
        _request = std::make_shared<sonia_common_ros2::srv::AiActivationService_Request>();

        auto front = getInput<AI_ACTIVATION_FRONT_TYPE>(AI_ACTIVATION_FRONT_NAME).value();
        auto bottom = getInput<AI_ACTIVATION_BOTTOM_TYPE>(AI_ACTIVATION_BOTTOM_NAME).value();
        auto model = getInput<AI_ACTIVATION_MODEL_TYPE>(AI_ACTIVATION_MODEL_NAME).value();

        RCLCPP_INFO(_ros_node->get_logger(), "The front response is %d and The bottom response is %d", front, bottom);

        // We activate the selected AI on the requested camera(s)
        if(front && bottom){
            _request->camera_choice=3;
            _request->model_choice=model;
            _ai_client->async_send_request(_request);
        }
        else if(!front && bottom){
            _request->camera_choice=2;
            _request->model_choice=model;
            _ai_client->async_send_request(_request);
        }
        else if(front && !bottom){
            _request->camera_choice=1;
            _request->model_choice=model;
            _ai_client->async_send_request(_request);
        }
        else if(!front && !bottom){
            _request->camera_choice=0;
            _request->model_choice=model;
            _ai_client->async_send_request(_request);
        }
        else
            return BT::NodeStatus::FAILURE;
        return BT::NodeStatus::SUCCESS;
        
    }    
}