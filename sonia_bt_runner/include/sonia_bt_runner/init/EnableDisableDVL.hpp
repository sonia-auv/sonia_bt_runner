#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "std_msgs/msg/bool.hpp"

#define ENABLE_DISABLE_DVL_INPUT "dvl_input"
#define ENABLE_DISABLE_DVL_INPUT_PARAMS ENABLE_DISABLE_DVL_INPUT, false, "true: start, false: stop"
#define ENABLE_DISABLE_DVL_INPUT_TYPE bool

namespace init{
    class EnableDisableDVL: public BT::SyncActionNode{
        public:
            EnableDisableDVL(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~EnableDisableDVL() override = default;
            static BT::PortsList providedPorts(){
                return {BT::InputPort<ENABLE_DISABLE_DVL_INPUT_TYPE>(ENABLE_DISABLE_DVL_INPUT_PARAMS)};
            }
            BT::NodeStatus tick() override;
            
        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _dvl_pub;
    };
} // namespace init
