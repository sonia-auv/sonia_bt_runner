#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "std_srvs/srv/trigger.hpp"

#define TARE_SENSOR "sensor"
#define TARE_SENSOR_PARAMS TARE_SENSOR, "imu or depth"
#define TARE_SENSOR_TYPE std::string

namespace init{
    class Tare: public BT::StatefulActionNode{
        public:
            Tare(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~Tare() override = default;
            static BT::PortsList providedPorts(){
                return {BT::InputPort<TARE_SENSOR_TYPE>(TARE_SENSOR_PARAMS)};
            }
            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;

        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
            std::shared_ptr<std_srvs::srv::Trigger_Request> _request;
            std::shared_ptr<std_srvs::srv::Trigger_Response> _response;
            rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _imu_client;
            rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _depth_client;
    };
} // namespace init
