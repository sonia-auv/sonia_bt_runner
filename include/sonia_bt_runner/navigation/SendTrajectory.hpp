#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "std_msgs/msg/int8.hpp"
#include "sonia_common_ros2/msg/pose.hpp"
#include "sonia_common_ros2/msg/pose_array.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"

using namespace std::chrono_literals;
namespace navigation{
    class SendTrajectory: public BT::StatefulActionNode{
        public:
            SendTrajectory(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~SendTrajectory();
            static BT::PortsList providedPorts()
            {
                const int def_inter = 0;
                return {
                    BT::InputPort<int>("interpolation", def_inter, ""),
                    BT::InputPort<Trajectory>("trajectory"),
                };
            }
            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;

        private:
            void isWaypointValid(const std_msgs::msg::Int8 &msg);

            std::shared_ptr<rclcpp::Node> ros_node;
            rclcpp::Publisher<sonia_common_ros2::msg::PoseArray>::SharedPtr planner_pub;
            rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr planner_sub;
            
            int valid;
            std::chrono::_V2::system_clock::time_point _time_launch;        
    };
}