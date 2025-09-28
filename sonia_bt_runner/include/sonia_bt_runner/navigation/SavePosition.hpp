#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sonia_common_ros2/msg/mission_timer.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace navigation{
    class SavePosition : public BT::SyncActionNode
    {
    public:
        SavePosition(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<Trajectory>("trajectory"),
            };
        }

        BT::NodeStatus tick() override;

    private:
        void get_position_callback(const geometry_msgs::msg::Pose &msg);

        std::tuple<double, double, double> quaternionToEuler(double w, double x, double y, double z);

        std::shared_ptr<rclcpp::Node> ros_node;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr position_sub;

        TrajectoryPose _tp;
        bool _saved = false;
    };
}