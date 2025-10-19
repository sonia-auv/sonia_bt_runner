#pragma once
#include <std_msgs/msg/float32.hpp>

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"


namespace sensors
{
    class DepthCheck : public BT::ConditionNode
    {
        public:
            DepthCheck(const std::string &name, std::shared_ptr<rclcpp::Node> node);
            ~DepthCheck() = default;
            static BT::PortsList providedPorts()
            {
                return
                {
                    BT::InputPort<float>("Target", 0.1, "true: ON, talse: OFF"),
                        BT::InputPort<bool>("IsSmaller", false, "true: ON, talse: OFF"),
                };
            }
            BT::NodeStatus tick() override;

        private:
            void update_status(const std_msgs::msg::Float32::ConstSharedPtr &msg);

            std::shared_ptr<rclcpp::Node> ros_node;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_sub;
            float _depth_val;
    };
}  // namespace sensors
