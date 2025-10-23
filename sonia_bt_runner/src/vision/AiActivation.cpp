#include "sonia_bt_runner/vision/AiActivation.hpp"

namespace vision{
    AiActivation::AiActivation(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::SyncActionNode(name, config), ros_node(node){
        ai_client = ros_node->create_client<sonia_common_ros2::srv::AiActivationService>("/proc_vision/ai_activation");
    }

    BT::NodeStatus AiActivation::tick(){
        std::shared_ptr<sonia_common_ros2::srv::AiActivationService_Response> response;
        request = std::make_shared<sonia_common_ros2::srv::AiActivationService_Request>();
        BT::Expected<bool> front = getInput<bool>("Front");
        BT::Expected<bool> bottom = getInput<bool>("Bottom");
        BT::Expected<int> model = getInput<int>("Model");

        if(front.value()&&bottom.value()){
            request->camera_choice=3;
            request->model_choice=model.value();
            ai_client->async_send_request(request);
        }
        else if(!front.value()&& bottom.value()){
            request->camera_choice=2;
            request->model_choice=model.value();
            ai_client->async_send_request(request);
        }
        else if(front.value()&& !bottom.value()){
            request->camera_choice=1;
            request->model_choice=model.value();
            ai_client->async_send_request(request);
        }
        else if(!front.value()&& !bottom.value()){
            request->camera_choice=0;
            request->model_choice=model.value();
            ai_client->async_send_request(request);
        }
        else
            return BT::NodeStatus::FAILURE;
        return BT::NodeStatus::SUCCESS;
        
    }    
}