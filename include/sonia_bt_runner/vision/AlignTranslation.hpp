#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"

namespace vision{
    class AlignTranslation : public BT::ConditionNode
    {
    public:
        AlignTranslation(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
        ~AlignTranslation()=default;
    private:
        std::shared_ptr<rclcpp::Node> ros_node;

        float depth;
    };
}//namespace vision