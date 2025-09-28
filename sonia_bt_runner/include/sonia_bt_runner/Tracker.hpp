#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include <behaviortree_cpp/loggers/abstract_logger.h>
#include "sonia_common_ros2/action/mission_control.hpp"

using MissionControl = sonia_common_ros2::action::MissionControl;
using GoalHandle = rclcpp_action::ServerGoalHandle<MissionControl>;

class Tracker : public BT::StatusChangeLogger
{
    public:
        Tracker(BT::Tree& tree, const std::shared_ptr<GoalHandle> goal);
        ~Tracker();

        void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status) override;
        void flush() override;
         
    private:
        std::shared_ptr<GoalHandle> gl;
        std::shared_ptr<MissionControl::Feedback> feedback;
    
};