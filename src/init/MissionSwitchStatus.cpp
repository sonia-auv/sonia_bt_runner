#include "sonia_bt_runner/init/MissionSwitchStatus.hpp"

using std::placeholders::_1;

namespace init{     

    MissionSwitchStatus::MissionSwitchStatus(const std::string &name, std::shared_ptr<rclcpp::Node> node)
    : BT::ConditionNode(name, {}), ros_node(node), _mission_status{false}
    {
        mission_sub= ros_node->create_subscription<sonia_common_ros2::msg::MissionStatus>("/provider_rs485/mission_status",10, std::bind(&MissionSwitchStatus::update_status,this, _1));
    }

    MissionSwitchStatus::~MissionSwitchStatus(){}

    BT::NodeStatus MissionSwitchStatus::tick()
    {
        if(_mission_status)
        {
            std::cout <<"succes"<<std::endl;
            return BT::NodeStatus::SUCCESS;
            
        }
        return BT::NodeStatus::FAILURE;
        
    }
    void MissionSwitchStatus::update_status(const sonia_common_ros2::msg::MissionStatus::ConstPtr &msg)
    {
        RCLCPP_INFO(ros_node->get_logger(), "status: %s", msg->status);
        _mission_status = msg->status;
        //std::cout <<"mission: "<<_mission_status<<std::endl;
    }
}
