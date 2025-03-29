#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "std_srvs/srv/trigger.hpp"

namespace init{
    class Tare: public BT::SyncActionNode{
        public:
            Tare(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~Tare();
            static BT::PortsList providedPorts(){
                return {BT::InputPort<std::string>("sensor", "imu or depth")};
            }

            BT::NodeStatus tick() override;
        private:
            
            std::shared_ptr<rclcpp::Node> ros_node;
            std::shared_ptr<std_srvs::srv::Trigger_Request> request;
            rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr imu_client;
            rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr depth_client;
    };
}