#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/bt_observer.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

class Logging: public BT::StatusChangeLogger
{
    public:
        Logging(BT::Tree& tree);

        void callback(BT::Duration timestamp,const BT::TreeNode& node, BT::NodeStatus prev, BT::NodeStatus status) override;
};