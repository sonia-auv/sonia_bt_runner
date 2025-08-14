#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"

namespace vision{
    class AlignRotation : public BT::SyncActionNode
    {
    public:
        AlignRotation(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
        ~AlignRotation();
    private:
        std::shared_ptr<rclcpp::Node> ros_node;
    };
}//namespace vision