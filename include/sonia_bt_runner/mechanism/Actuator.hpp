#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/srv/dropper_service.hpp"

namespace mechanism{
    class Actuator: public BT::StatefulActionNode{
        public:
            Actuator(const std::string &name, const BT::NodeConfig &config);
            ~Actuator();
            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;

        private:
            std::shared_ptr<rclcpp::Node> ros_node;
            //rclcpp::Publisher<sonia_common_ros2::srv::DropperService>::SharedPtr dropper_pub;
            //rclcpp::Subscription<sonia_common_ros2::msg::Pose>::SharedPtr planner_sub;
        
    };
}