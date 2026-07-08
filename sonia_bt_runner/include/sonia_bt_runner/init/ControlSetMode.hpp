#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/msg/mpc_info.hpp"
#include "std_msgs/msg/u_int8.hpp"

#define CONTROL_SET_MODE_SET_MODE "set_mode"
#define CONTROL_SET_MODE_SET_MODE_PARAMS CONTROL_SET_MODE_SET_MODE, false, ""
#define CONTROL_SET_MODE_SET_MODE_TYPE bool

namespace init{
    class ControlSetMode: public BT::StatefulActionNode{
        public:
            ControlSetMode(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~ControlSetMode() override = default;
            static BT::PortsList providedPorts(){
                return {BT::InputPort<CONTROL_SET_MODE_SET_MODE_TYPE>(CONTROL_SET_MODE_SET_MODE_PARAMS)};
            }
            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;

        private:
            void control_callback(const sonia_common_ros2::msg::MpcInfo &msg);
            
            std::shared_ptr<rclcpp::Node> _ros_node;
            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr _mode_pub;
            rclcpp::Subscription<sonia_common_ros2::msg::MpcInfo>::SharedPtr _ctrl_sub;
            uint8_t _ctrl_state;
    };
} // namespace init
