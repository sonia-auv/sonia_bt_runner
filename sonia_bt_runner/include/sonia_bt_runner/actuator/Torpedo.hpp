#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/srv/actuator_service.hpp"

namespace actuator{
    class Torpedo: public BT::StatefulActionNode{
        public:
            Torpedo(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~Torpedo() = default;
            static BT::PortsList providedPorts()
            {
                // Options for 'side' are 'port_side' or 'starboard'
                return {BT::InputPort<std::string>("side", "port_side or starboard")};
            }
            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;

        private:
            std::shared_ptr<rclcpp::Node> ros_node;
            std::shared_ptr<sonia_common_ros2::srv::ActuatorService::Request> request;
            rclcpp::Client<sonia_common_ros2::srv::ActuatorService>::SharedPtr torpedo_client;        
    };
}