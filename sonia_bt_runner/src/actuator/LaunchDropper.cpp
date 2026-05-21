#include "sonia_bt_runner/actuator/LaunchDropper.hpp"

namespace actuator{
    LaunchDropper::LaunchDropper(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), _ros_node(node)
    {
        _dropper_client = _ros_node->create_client<sonia_common_ros2::srv::ActuatorService>("/provider_actuator/do_action");
    }

    BT::NodeStatus LaunchDropper::onStart(){
        _request = std::make_shared<sonia_common_ros2::srv::ActuatorService::Request>();
        _request->action= _request->ACTION_LAUNCH;
        _request->element= _request->ELEMENT_DROPPER;
        BT::Expected<std::string> side(getInput<std::string>("side"));
        if(side = "port_side")
        {
            _request->side = _request->SIDE_PORT;
        }
        if(side = "starboard")
        {
            _request->side = _request->SIDE_STARBOARD;
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus LaunchDropper::onRunning(){
        auto response = _dropper_client->async_send_request(_request);
        if(response.get()->success){
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
    
    void LaunchDropper::onHalted(){}

}