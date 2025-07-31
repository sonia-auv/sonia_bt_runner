#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include <behaviortree_cpp/loggers/abstract_logger.h>
#include "sonia_common_ros2/action/mission_control.hpp"

using MissionControl = sonia_common_ros2::action::MissionControl;
class Tracker : public BT::StatusChangeLogger
{
    public:
        Tracker(BT::Tree& tree, std::shared_ptr<MissionControl::Feedback> feedback);
        ~Tracker();

        void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status) override;
        void flush() override;
        
        std::shared_ptr<MissionControl::Feedback> fb;
};