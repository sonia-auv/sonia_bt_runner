#include "sonia_bt_runner/startUp/MissionSwitchStatus.hpp"

using std::placeholders::_1;

namespace start_up{     

    MissionSwitchStatus::MissionSwitchStatus(const std::string &name): BT::ConditionNode(name, {}), Node(name)
    {
        _subscriberMissionStatus= this->create_subscription<sonia_common_ros2::msg::MissionStatus>("provider_rs485/mission_status",10, std::bind(&MissionSwitchStatus::update_status, this, _1));
    }

    MissionSwitchStatus::~MissionSwitchStatus(){}

    BT::NodeStatus MissionSwitchStatus::tick()
    {
        if(_mission_status)
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
    void MissionSwitchStatus::update_status(const sonia_common_ros2::msg::MissionStatus::ConstPtr &msg)
    {
        _mission_status = msg->status;
    }
}
