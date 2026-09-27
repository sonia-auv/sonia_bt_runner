#pragma once
#include <std_msgs/msg/float32.hpp>

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

#define DEPTH_CHECK_TARGET "Target"
#define DEPTH_CHECK_TARGET_PARAMS DEPTH_CHECK_TARGET, 0.1, "true: ON, talse: OFF"
#define DEPTH_CHECK_TARGET_TYPE float

#define DEPTH_CHECK_IS_SMALLER "IsSmaller"
#define DEPTH_CHECK_IS_SMALLER_PARAMS DEPTH_CHECK_IS_SMALLER, false, "true: ON, talse: OFF"
#define DEPTH_CHECK_IS_SMALLER_TYPE bool

namespace sensors
{
    class DepthCheck : public BT::ConditionNode
    {
        public:
            DepthCheck(const std::string &name, std::shared_ptr<rclcpp::Node> node);
            ~DepthCheck() override = default;
            static BT::PortsList providedPorts()
            {
                return
                {
                    BT::InputPort<DEPTH_CHECK_TARGET_TYPE>(DEPTH_CHECK_TARGET_PARAMS),
                    BT::InputPort<DEPTH_CHECK_IS_SMALLER_TYPE>(DEPTH_CHECK_IS_SMALLER_PARAMS),
                };
            }
            BT::NodeStatus tick() override;

        private:
            void update_status(const std_msgs::msg::Float32::ConstSharedPtr &msg);

            std::shared_ptr<rclcpp::Node> _ros_node;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _depth_sub;
            float _depth_val;
    };
}  // namespace sensors
