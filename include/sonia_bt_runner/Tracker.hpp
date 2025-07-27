#pragma once

#include <behaviortree_cpp/loggers/abstract_logger.h>

class Tracker : public BT::StatusChangeLogger
{
    public:
        Tracker(BT::Tree& tree);

        void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status) override;
};