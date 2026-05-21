#include "sonia_bt_runner/init/Tare.hpp"

namespace init{

    Tare::Tare(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), _ros_node(node)
    {
        _request = std::make_shared<std_srvs::srv::Trigger_Request>();
        _imu_client = _ros_node->create_client<std_srvs::srv::Trigger>("provider_imu/tare");
        _depth_client = _ros_node->create_client<std_srvs::srv::Trigger>("provider_depth/tare");
    }

    BT::NodeStatus Tare::onStart()
    {
        BT::Expected<std::string> sensor(getInput<std::string>("sensor"));
        if(sensor.value()=="imu"){
            _response= _imu_client->async_send_request(_request).get();
        }
        if(sensor.value()=="depth"){
            _response = _depth_client->async_send_request(_request).get();
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus Tare::onRunning()
    {
        if(_response->success){
            RCLCPP_INFO(_ros_node->get_logger(),"%s", _response->message.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void Tare::onHalted()
    {
    }
} // namespace init