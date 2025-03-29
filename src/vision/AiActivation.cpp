#include "sonia_bt_runner/vision/AiActivation.hpp"

namespace vision{
    AiActivation::AiActivation(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::SyncActionNode(name, config), ros_node(node){
        ai_client = ros_node->create_client<sonia_common_ros2::srv::AiActivationService>("/proc_vision/ai_activation");

    }
    AiActivation::~AiActivation(){}

    BT::NodeStatus AiActivation::tick(){
        std::shared_ptr<sonia_common_ros2::srv::AiActivationService_Response> response;
        request = std::make_shared<sonia_common_ros2::srv::AiActivationService_Request>();
        BT::Expected<bool> front = getInput<bool>("Front");
        BT::Expected<bool> bottom = getInput<bool>("Bottom");
        if(front.value()&&bottom.value()){
            request->ai_activation=3;
            ai_client->async_send_request(request);
        }
        else if(!front.value()&& bottom.value()){
            request->ai_activation=2;
            ai_client->async_send_request(request);
        }
        else if(front.value()&& !bottom.value()){
            request->ai_activation=1;
            ai_client->async_send_request(request);
        }
        else if(!front.value()&& !bottom.value()){
            request->ai_activation=0;
            ai_client->async_send_request(request);
        }
        else
            return BT::NodeStatus::FAILURE;
        return BT::NodeStatus::SUCCESS;
        
    }    
}