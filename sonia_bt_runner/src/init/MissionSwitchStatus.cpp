#include "sonia_bt_runner/init/MissionSwitchStatus.hpp"

using std::placeholders::_1;

namespace init{     

    MissionSwitchStatus::MissionSwitchStatus(const std::string &name, std::shared_ptr<rclcpp::Node> node)
    : BT::ConditionNode(name, {}), ros_node(node), _mission_status{false}
    {
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        mission_sub= ros_node->create_subscription<sonia_common_ros2::msg::MissionStatus>("/provider_rs485/mission_status", qos, std::bind(&MissionSwitchStatus::update_status,this, _1));
    }

    BT::NodeStatus MissionSwitchStatus::tick()
    {
        if(_mission_status)
        {
            return BT::NodeStatus::SUCCESS;   
        }
        return BT::NodeStatus::FAILURE;
        
    }
    void MissionSwitchStatus::update_status(const sonia_common_ros2::msg::MissionStatus::ConstSharedPtr &msg)
    {
        _mission_status = msg->status;
    }
} // namespace init
