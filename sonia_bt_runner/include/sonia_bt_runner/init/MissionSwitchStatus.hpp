#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/msg/mission_status.hpp"
#include "std_msgs/msg/bool.hpp"

namespace init{
    class MissionSwitchStatus: public BT::ConditionNode
    {
        public:
            MissionSwitchStatus(const std::string &name, std::shared_ptr<rclcpp::Node> node);
            ~MissionSwitchStatus() override = default;
            BT::NodeStatus tick() override;
            
        private:
            void update_status(const sonia_common_ros2::msg::MissionStatus::ConstSharedPtr &msg);
            void update_simulation_status(const std_msgs::msg::Bool::ConstSharedPtr &msg);

            std::shared_ptr<rclcpp::Node> _ros_node;
            rclcpp::Subscription<sonia_common_ros2::msg::MissionStatus>::SharedPtr _mission_sub;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _simulation_sub;
            bool _mission_status;
            bool _in_simulation;
    };
    
} // namespace init
