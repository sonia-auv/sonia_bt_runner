#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/srv/actuator_service.hpp"

#define LAUNCH_DROPPER_SIDE "side"
#define LAUNCH_DROPPER_SIDE_PARAMS LAUNCH_DROPPER_SIDE, "port_side or starboard"
#define LAUNCH_DROPPER_SIDE_TYPE std::string

namespace actuator{
    class LaunchDropper: public BT::StatefulActionNode{
        public:
            LaunchDropper(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~LaunchDropper() override = default;
            static BT::PortsList providedPorts()
            {
                // Options for 'side' are 'port_side' or 'starboard'
                return {BT::InputPort<LAUNCH_DROPPER_SIDE_TYPE>(LAUNCH_DROPPER_SIDE_PARAMS)};
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
