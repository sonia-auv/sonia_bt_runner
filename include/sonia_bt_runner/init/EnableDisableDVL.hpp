#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "std_msgs/msg/bool.hpp"

namespace init{
    class EnableDisableDVL: public BT::SyncActionNode{
        public:
            EnableDisableDVL(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~EnableDisableDVL();
            static BT::PortsList providedPorts(){
                return {BT::InputPort<bool>("dvl_input",true, "")};
            }
            BT::NodeStatus tick() override;
        private:
            std::shared_ptr<rclcpp::Node> ros_node;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dvl_pub;
    };
}