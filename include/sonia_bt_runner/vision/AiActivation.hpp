/*#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"

namespace vision{
    class AiActivation : public BT::SyncActionNode
    {
    public:
        AiActivation(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
        ~AiActivation();

        static BT::PortsList providedPorts()
        {
            const int activation_target_def = 0;
            return {BT::InputPort<int>("activation_target", activation_target_def, "0: OFF, 1: FRONT, 2: BOTTOM, 3: BOTH")};
        }

        BT::NodeStatus tick() override;

    private:
        std::shared_ptr<rclcpp::Node> ros_node;
    };
}*/