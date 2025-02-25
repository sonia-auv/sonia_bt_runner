#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/msg/pose.hpp"
#include "sonia_common_ros2/msg/pose_array.hpp"

namespace navigation{
    class SendWaypoint: public BT::StatefulActionNode{
        public:
            SendWaypoint(const std::string &name, const BT::NodeConfig &config);
            ~SendWaypoint();
            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;

        private:
            std::shared_ptr<rclcpp::Node> ros_node;
            rclcpp::Publisher<sonia_common_ros2::msg::Pose>::SharedPtr planner_pub;
            rclcpp::Subscription<sonia_common_ros2::msg::Pose>::SharedPtr planner_sub;
        
    };
}