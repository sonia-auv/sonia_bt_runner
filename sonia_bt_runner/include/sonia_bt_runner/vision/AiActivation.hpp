#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/srv/ai_activation_service.hpp"

#define AI_ACTIVATION_FRONT "Front"
#define AI_ACTIVATION_FRONT_PARAMS AI_ACTIVATION_FRONT, false, "true: ON, talse: OFF"
#define AI_ACTIVATION_FRONT_TYPE bool

#define AI_ACTIVATION_BOTTOM "Bottom"
#define AI_ACTIVATION_BOTTOM_PARAMS AI_ACTIVATION_BOTTOM, false, "true: ON, talse: OFF"
#define AI_ACTIVATION_BOTTOM_TYPE bool

#define AI_ACTIVATION_MODEL "Model"
#define AI_ACTIVATION_MODEL_PARAMS AI_ACTIVATION_MODEL, 0, "model choice, default = 0"
#define AI_ACTIVATION_MODEL_TYPE int

namespace vision{
    class AiActivation : public BT::SyncActionNode
    {
    public:
        AiActivation(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
        ~AiActivation() override = default;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<AI_ACTIVATION_FRONT_TYPE>(AI_ACTIVATION_FRONT_PARAMS),
                BT::InputPort<AI_ACTIVATION_BOTTOM_TYPE>(AI_ACTIVATION_BOTTOM_PARAMS),
                BT::InputPort<AI_ACTIVATION_MODEL_TYPE>(AI_ACTIVATION_MODEL_PARAMS)};
        }

        BT::NodeStatus tick() override;

    private:
        std::shared_ptr<sonia_common_ros2::srv::AiActivationService_Request> _request;
        rclcpp::Client<sonia_common_ros2::srv::AiActivationService>::SharedPtr _ai_client;
        std::shared_ptr<rclcpp::Node> _ros_node;
    };
}
