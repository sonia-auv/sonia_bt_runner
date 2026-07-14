#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/Point.hpp"

#include <geometry_msgs/msg/pose.hpp>

#define SAVE_POINT_POINT "Point"
#define SAVE_POINT_POINT_TYPE Point

namespace navigation {

    class SavePoint : public BT::StatefulActionNode {

            public:
            
            SavePoint(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~SavePoint() override = default;

            static BT::PortsList providedPorts()
            {
                return {
                    BT::OutputPort<SAVE_POINT_POINT_TYPE>(SAVE_POINT_POINT),
                };
            }

            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;
            void pose_call_back(const geometry_msgs::msg::Pose &msg);

        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
            rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _pose_sub;
            geometry_msgs::msg::Pose _pose_msg;
            std::chrono::_V2::system_clock::time_point _launch_time;
            float _time_diff;
            bool _msg_received;
    };
}
