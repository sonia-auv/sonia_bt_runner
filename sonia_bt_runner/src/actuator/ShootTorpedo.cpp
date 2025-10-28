#include "sonia_bt_runner/actuator/ShootTorpedo.hpp"

namespace actuator{
    ShootTorpedo::ShootTorpedo(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), ros_node(node)
    {
        torpedo_client = ros_node->create_client<sonia_common_ros2::srv::ActuatorService>("/provider_actuator/do_action");
    }
    ShootTorpedo::~ShootTorpedo(){}

    BT::NodeStatus ShootTorpedo::onStart(){
        request = std::make_shared<sonia_common_ros2::srv::ActuatorService::Request>();
        request->action= request->ACTION_LAUNCH;
        request->element= request->ELEMENT_TORPEDO;
        if(getInput<std::string>("side") = "port_side"){
            request->side = request->SIDE_PORT;
        }
        if(getInput<std::string>("side") = "starboard"){
            request->side = request->SIDE_STARBOARD;
        }
        return BT::NodeStatus::RUNNING;
        
    }
    BT::NodeStatus ShootTorpedo::onRunning(){
        auto response = torpedo_client->async_send_request(request);
        if(response.get()->success){
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;

    }
    void ShootTorpedo::onHalted(){}
}