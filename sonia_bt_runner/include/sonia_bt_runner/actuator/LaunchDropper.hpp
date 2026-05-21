#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/srv/actuator_service.hpp"

namespace actuator{
    class LaunchDropper: public BT::StatefulActionNode{
        public:
            LaunchDropper(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~LaunchDropper() override = default;
            static BT::PortsList providedPorts()
            {
                // Options for 'side' are 'port_side' or 'starboard'
                return {BT::InputPort<std::string>("side", "port_side or starboard")};
            }
            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;

        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
            std::shared_ptr<sonia_common_ros2::srv::ActuatorService::Request> _request;
            rclcpp::Client<sonia_common_ros2::srv::ActuatorService>::SharedPtr _dropper_client;
    };
}