#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/Point.hpp"
#include "sonia_common_ros2/msg/pose.hpp"

namespace navigation {

    class SavePoint : public BT::StatefulActionNode {

            public:
            
            SavePoint(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~SavePoint() override = default;

            static BT::PortsList providedPorts()
            {
                return {
                    BT::OutputPort<Point>("Point"),
                };
            }

            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;
            void pose_call_back(const sonia_common_ros2::msg::Pose &msg);

        private:
            std::shared_ptr<rclcpp::Node> _ros_node;
            rclcpp::Subscription<sonia_common_ros2::msg::Pose>::SharedPtr _pose_sub;
            sonia_common_ros2::msg::Pose _pose_msg;
            std::chrono::_V2::system_clock::time_point _launch_time;
            float _time_diff;
            bool _msg_received;
    };
}