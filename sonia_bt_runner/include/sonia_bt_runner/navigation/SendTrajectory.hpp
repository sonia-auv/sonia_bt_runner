#pragma once

#include <std_msgs/msg/float32.hpp>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "std_msgs/msg/int8.hpp"
#include "sonia_common_ros2/msg/pose.hpp"
#include "sonia_common_ros2/msg/pose_array.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"


#define SEND_TRAJECTORY_INTERPOLATION_PARAM_NAME "interpolation"
#define SEND_TRAJECTORY_INTERPOLATION_PARAM_TYPE int
#define SEND_TRAJECTORY_TRAJECTORY_PARAM_NAME "trajectory"
#define SEND_TRAJECTORY_TRAJECTORY_PARAM_TYPE Trajectory

using namespace std::chrono_literals;
namespace navigation{
    class SendTrajectory: public BT::StatefulActionNode{
        public:
            SendTrajectory(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~SendTrajectory() override = default;
            static BT::PortsList providedPorts()
            {
                const int def_inter = 0;
                return {
                    BT::InputPort<SEND_TRAJECTORY_INTERPOLATION_PARAM_TYPE>(SEND_TRAJECTORY_INTERPOLATION_PARAM_NAME, def_inter, ""),
                    BT::InputPort<SEND_TRAJECTORY_TRAJECTORY_PARAM_TYPE>(SEND_TRAJECTORY_TRAJECTORY_PARAM_NAME),
                };
            }
            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;

        private:
            void isWaypointValid(const std_msgs::msg::Int8 &msg);
            void update_depth(const std_msgs::msg::Float32::ConstSharedPtr &msg);

            std::shared_ptr<rclcpp::Node> _ros_node;
            rclcpp::Publisher<sonia_common_ros2::msg::PoseArray>::SharedPtr _planner_pub;
            rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr _planner_sub;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _depth_sub;
            int _valid;
            std::chrono::_V2::system_clock::time_point _time_launch;
            float _depth_val;

    };
}