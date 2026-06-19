#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/srv/ai_activation_service.hpp"

#define AI_ACTIVATION_FRONT_PARAM_NAME "Front"
#define AI_ACTIVATION_FRONT_PARAM_TYPE bool
#define AI_ACTIVATION_BOTTOM_PARAM_NAME "Bottom"
#define AI_ACTIVATION_BOTTOM_PARAM_TYPE bool
#define AI_ACTIVATION_MODEL_PARAM_NAME "Model"
#define AI_ACTIVATION_MODEL_PARAM_TYPE int

namespace vision{
    class AiActivation : public BT::SyncActionNode
    {
    public:
        AiActivation(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
        ~AiActivation() override = default;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<AI_ACTIVATION_FRONT_PARAM_TYPE>(AI_ACTIVATION_FRONT_PARAM_NAME, false, "true: ON, talse: OFF"),
                BT::InputPort<AI_ACTIVATION_BOTTOM_PARAM_TYPE>(AI_ACTIVATION_BOTTOM_PARAM_NAME, false, "true: ON, talse: OFF"),
                BT::InputPort<AI_ACTIVATION_MODEL_PARAM_TYPE>(AI_ACTIVATION_MODEL_PARAM_NAME, 0, "model choice, default = 0")};
        }

        BT::NodeStatus tick() override;

    private:
        std::shared_ptr<sonia_common_ros2::srv::AiActivationService_Request> _request;
        rclcpp::Client<sonia_common_ros2::srv::AiActivationService>::SharedPtr _ai_client;
        std::shared_ptr<rclcpp::Node> _ros_node;
    };
}