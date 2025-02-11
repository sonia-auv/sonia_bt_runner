#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/msg/mission_status.hpp"

namespace init{
    class MissionSwitchStatus: public BT::ConditionNode, public rclcpp::Node
    {
        public:
            MissionSwitchStatus(const std::string &name);
            ~MissionSwitchStatus();
            BT::NodeStatus tick() override;
            
        private:
            void update_status(const sonia_common_ros2::msg::MissionStatus::ConstPtr &msg);
            rclcpp::Subscription<sonia_common_ros2::msg::MissionStatus>::SharedPtr _subscriberMissionStatus;
            bool _mission_status;
    };
    
}
