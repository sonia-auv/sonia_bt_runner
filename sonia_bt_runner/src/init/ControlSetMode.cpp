#include "sonia_bt_runner/init/ControlSetMode.hpp"
using std::placeholders::_1;
namespace init{
    ControlSetMode::ControlSetMode(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), _ros_node(node)
    {
        _mode_pub = _ros_node->create_publisher<std_msgs::msg::UInt8>("/proc_control/set_mode",100);
    }

    BT::NodeStatus ControlSetMode::onStart(){

        std_msgs::msg::UInt8 msg;
        
        BT::Expected<CONTROL_SET_MODE_SET_MODE_TYPE> data(getInput<CONTROL_SET_MODE_SET_MODE_TYPE>(CONTROL_SET_MODE_SET_MODE));
        if(data.value()){
            msg.data = 10;
            _mode_pub->publish(msg);
        }
        else{
            msg.data = 0;
            _mode_pub->publish(msg);
        }

        _ctrl_sub = _ros_node->create_subscription<sonia_common_ros2::msg::MpcInfo>("/proc_control/controller_info",1, std::bind(&ControlSetMode::control_callback, this, _1));
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus ControlSetMode::onRunning(){
        if(_ctrl_state == 10 || _ctrl_state == 0){
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
    void ControlSetMode::onHalted()
    {
    }
    void ControlSetMode::control_callback(const sonia_common_ros2::msg::MpcInfo &msg){
        _ctrl_state=msg.mpc_mode;
    }
} // namespace init
