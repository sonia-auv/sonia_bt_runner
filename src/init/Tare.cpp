#include "sonia_bt_runner/init/Tare.hpp"

namespace init{

    Tare::Tare(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), ros_node(node)
    {
        imu_client = ros_node->create_client<std_srvs::srv::Trigger>("provider_imu/tare");
        depth_client = ros_node->create_client<std_srvs::srv::Trigger>("provider_depth/tare");
    }
    Tare::~Tare(){}
    BT::NodeStatus Tare::tick()
    {
        std::shared_ptr<std_srvs::srv::Trigger_Response> response;
        request = std::make_shared<std_srvs::srv::Trigger_Request>();
        BT::Expected<std::string> sensor = getInput<std::string>("sensor");
        if(sensor.value()=="imu"){
            response= imu_client->async_send_request(request).get();
        }
        if(sensor.value()=="depth"){
            response = depth_client->async_send_request(request).get();
        }
        if(response->success){
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

}