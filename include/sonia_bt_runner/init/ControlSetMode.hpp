#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/msg/mpc_info.hpp"

namespace init{
    class ControlSetMode: public BT::StatefulActionNode{
        public:
            ControlSetMode(const std::string &name, const BT::NodeConfig &config);
            ~ControlSetMode();
            static BT::PortsList providedPorts(){
                return {BT::InputPort<bool>("set_mode", false,"")};
            }

            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;
        private:
            void control_callback(const sonia_common_ros2::msg::MpcInfo &msg);
            std::shared_ptr<rclcpp::Node> ros_node;
            rclcpp::Subscription<sonia_common_ros2::msg::MpcInfo>::SharedPtr mission_sub;
    };
}