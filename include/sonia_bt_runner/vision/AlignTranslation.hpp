#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"

namespace vision{
    class AlignTranslation : public BT::SyncActionNode
    {
    public:
        AlignTranslation(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
        ~AlignTranslation();
    private:
        std::shared_ptr<rclcpp::Node> ros_node;
    };
}//namespace vision