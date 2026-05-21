#include "sonia_bt_runner/vision/AiActivation.hpp"

namespace vision{
    AiActivation::AiActivation(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::SyncActionNode(name, config), _ros_node(node){
        _ai_client = _ros_node->create_client<sonia_common_ros2::srv::AiActivationService>("/proc_vision/ai_activation");
    }

    BT::NodeStatus AiActivation::tick(){
        std::shared_ptr<sonia_common_ros2::srv::AiActivationService_Response> response;
        _request = std::make_shared<sonia_common_ros2::srv::AiActivationService_Request>();
        BT::Expected<bool> front = getInput<bool>("Front");
        BT::Expected<bool> bottom = getInput<bool>("Bottom");
        BT::Expected<int> model = getInput<int>("Model");

        // We activate the selected AI on the requested camera(s)
        if(front.value()&&bottom.value()){
            _request->camera_choice=3;
            _request->model_choice=model.value();
            _ai_client->async_send_request(_request);
        }
        else if(!front.value()&& bottom.value()){
            _request->camera_choice=2;
            _request->model_choice=model.value();
            _ai_client->async_send_request(_request);
        }
        else if(front.value()&& !bottom.value()){
            _request->camera_choice=1;
            _request->model_choice=model.value();
            _ai_client->async_send_request(_request);
        }
        else if(!front.value()&& !bottom.value()){
            _request->camera_choice=0;
            _request->model_choice=model.value();
            _ai_client->async_send_request(_request);
        }
        else
            return BT::NodeStatus::FAILURE;
        return BT::NodeStatus::SUCCESS;
        
    }    
}