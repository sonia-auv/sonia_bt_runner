#include "sonia_bt_runner/actuator/LaunchDropper.hpp"

namespace actuator{
    LaunchDropper::LaunchDropper(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), ros_node(node)
    {
        dropper_client = ros_node->create_client<sonia_common_ros2::srv::ActuatorService>("/provider_actuator/do_action");
    }

    BT::NodeStatus LaunchDropper::onStart(){
        request = std::make_shared<sonia_common_ros2::srv::ActuatorService::Request>();
        request->action= request->ACTION_LAUNCH;
        request->element= request->ELEMENT_DROPPER;
        if(getInput<std::string>("side") = "port_side"){
            request->side = request->SIDE_PORT;
        }
        if(getInput<std::string>("side") = "starboard"){
            request->side = request->SIDE_STARBOARD;
        }

        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus LaunchDropper::onRunning(){
        auto response = dropper_client->async_send_request(request);
        if(response.get()->success){
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
    void LaunchDropper::onHalted(){}

}