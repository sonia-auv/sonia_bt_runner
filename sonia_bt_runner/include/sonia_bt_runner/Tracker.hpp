#pragma once

#include <chrono>
#include <rclcpp_action/rclcpp_action.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include <behaviortree_cpp/loggers/abstract_logger.h>
#include "sonia_common_ros2/action/mission_control.hpp"

using MissionControl = sonia_common_ros2::action::MissionControl;
using GoalHandle = rclcpp_action::ServerGoalHandle<MissionControl>;

/**
 * @class Tracker
 * @brief Tracks every ticks sent down the running tree.
 */
class Tracker : public BT::StatusChangeLogger
{
    public:
        Tracker(BT::Tree& tree, const std::shared_ptr<GoalHandle> goal);
        ~Tracker() override = default;

        /**
         * @brief Process information returned by the subscriber about the node's status.
         * @param timestamp Time of the event.
         * @param node The node that changed status.
         * @param prev_status The node's previous status.
         * @param status The node's new status.
         */
        void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status) override;

        /**
         * @brief Ensures no pending log data is left in the buffers.
         */
        void flush() override;
    
    private:
        std::shared_ptr<GoalHandle> _gl;
        std::shared_ptr<MissionControl::Feedback> _feedback;

        const uint8_t _THREAD_SLEEP_TIME = 66;
    
};