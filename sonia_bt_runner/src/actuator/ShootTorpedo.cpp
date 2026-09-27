#include "sonia_bt_runner/actuator/ShootTorpedo.hpp"

namespace actuator{
    ShootTorpedo::ShootTorpedo(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), _ros_node(node)
    {
        _torpedo_client = _ros_node->create_client<sonia_common_ros2::srv::ActuatorService>("/provider_actuator/do_action");
    }

    BT::NodeStatus ShootTorpedo::onStart(){
        _request = std::make_shared<sonia_common_ros2::srv::ActuatorService::Request>();
        _request->action= _request->ACTION_LAUNCH;
        _request->element= _request->ELEMENT_TORPEDO;
        BT::Expected<SHOOT_TORPEDO_SIDE_TYPE> side(getInput<SHOOT_TORPEDO_SIDE_TYPE>(SHOOT_TORPEDO_SIDE));
        if(side == "port_side"){
            _request->side = _request->SIDE_PORT;
            RCLCPP_INFO(_ros_node->get_logger(), "PortSide torpedo has been shot!");
        }
        if(side == "starboard"){
            _request->side = _request->SIDE_STARBOARD;
            RCLCPP_INFO(_ros_node->get_logger(), "Starboard torpedo has been shot!");
        }
        return BT::NodeStatus::RUNNING;
        
    }
    BT::NodeStatus ShootTorpedo::onRunning(){
        auto response = _torpedo_client->async_send_request(_request);
        if(response.get()->success){
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;

    }
    void ShootTorpedo::onHalted(){}
}
