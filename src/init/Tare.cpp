#include "sonia_bt_runner/init/Tare.hpp"

namespace init{

    Tare::Tare(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), ros_node(node)
    {
        request = std::make_shared<std_srvs::srv::Trigger_Request>();
        imu_client = ros_node->create_client<std_srvs::srv::Trigger>("provider_imu/tare");
        depth_client = ros_node->create_client<std_srvs::srv::Trigger>("provider_depth/tare");
    }
    Tare::~Tare(){}
    BT::NodeStatus Tare::tick()
    {
        BT::Expected<std::string> sensor = getInput<std::string>("sensor");
        if(sensor.value()=="imu"){
            response= imu_client->async_send_request(request).get();
            RCLCPP_INFO(ros_node->get_logger(), "tared imu");
        }
        if(sensor=="depth"){
            response = depth_client->async_send_request(request).get();
            RCLCPP_INFO(ros_node->get_logger(), "tared depth");
        }
        if(response->success){
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

}