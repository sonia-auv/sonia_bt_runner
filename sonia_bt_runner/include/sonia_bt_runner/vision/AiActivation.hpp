#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/srv/ai_activation_service.hpp"

namespace vision{
    class AiActivation : public BT::SyncActionNode
    {
    public:
        AiActivation(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
        ~AiActivation() override = default;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("Front", false, "true: ON, talse: OFF"),
                BT::InputPort<bool>("Bottom", false, "true: ON, talse: OFF"),
                BT::InputPort<int>("Model", 0, "model choice, default = 0")};
        }

        BT::NodeStatus tick() override;

    private:
        std::shared_ptr<sonia_common_ros2::srv::AiActivationService_Request> request;
        rclcpp::Client<sonia_common_ros2::srv::AiActivationService>::SharedPtr ai_client;
        std::shared_ptr<rclcpp::Node> ros_node;
    };
}