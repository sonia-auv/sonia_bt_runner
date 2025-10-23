#include "sonia_bt_runner/init/ControlSetMode.hpp"
using std::placeholders::_1;
namespace init{
    ControlSetMode::ControlSetMode(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), ros_node(node)
    {
        mode_pub = ros_node->create_publisher<std_msgs::msg::UInt8>("/proc_control/set_mode",100);
    }

    BT::NodeStatus ControlSetMode::onStart(){

        std_msgs::msg::UInt8 msg;
        
        BT::Expected<bool> data =getInput<bool>("set_mode");
        if(data.value()){
            msg.data = 10;
            mode_pub->publish(msg);
        }
        else{
            msg.data = 0;
            mode_pub->publish(msg);
        }

        ctrl_sub = ros_node->create_subscription<sonia_common_ros2::msg::MpcInfo>("/proc_control/controller_info",1, std::bind(&ControlSetMode::control_callback, this, _1));
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus ControlSetMode::onRunning(){
        if(ctrl_state == 10 || ctrl_state == 0){
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
    void ControlSetMode::onHalted()
    {
    }
    void ControlSetMode::control_callback(const sonia_common_ros2::msg::MpcInfo &msg){
        ctrl_state=msg.mpc_mode;
    }
} // namespace init