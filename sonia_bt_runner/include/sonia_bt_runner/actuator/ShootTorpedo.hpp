#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/srv/actuator_service.hpp"

#define SHOOT_TORPEDO_SIDE "side"
#define SHOOT_TORPEDO_SIDE_PARAMS SHOOT_TORPEDO_SIDE, "port_side or starboard"
#define SHOOT_TORPEDO_SIDE_TYPE std::string

namespace actuator{
    class ShootTorpedo: public BT::StatefulActionNode{
        public:
            ShootTorpedo(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~ShootTorpedo() override = default;
            static BT::PortsList providedPorts()
            {
                // Options for 'side' are 'port_side' or 'starboard'
                return {BT::InputPort<SHOOT_TORPEDO_SIDE_TYPE>(SHOOT_TORPEDO_SIDE_PARAMS)};
            }
            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;

        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
            std::shared_ptr<sonia_common_ros2::srv::ActuatorService::Request> _request;
            rclcpp::Client<sonia_common_ros2::srv::ActuatorService>::SharedPtr _torpedo_client;

            BT::Expected<SHOOT_TORPEDO_SIDE_TYPE> _side;
    };
}
