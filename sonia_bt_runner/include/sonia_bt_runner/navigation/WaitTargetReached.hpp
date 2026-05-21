#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "sonia_common_ros2/msg/mission_timer.hpp"
#include "sonia_common_ros2/msg/mpc_info.hpp"

namespace navigation{
    class WaitTargetReached : public BT::StatefulActionNode
    {
    public:
        WaitTargetReached(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
        ~WaitTargetReached() override = default;
        static BT::PortsList providedPorts()
        {
            const int def_timeout = 5;
            return {
                BT::InputPort<int>("timeout", def_timeout, "")
            };
        }
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        void get_controller_info_callback(const sonia_common_ros2::msg::MpcInfo &msg);
        sonia_common_ros2::msg::MissionTimer missionTimerFunc(std::string mission, float timeout, time_t uniqueID, int status);
        std::shared_ptr<rclcpp::Node> _ros_node;
        rclcpp::Publisher<sonia_common_ros2::msg::MissionTimer>::SharedPtr _timeout_pub;
        rclcpp::Subscription<sonia_common_ros2::msg::MpcInfo>::SharedPtr _controller_sub;
        std::chrono::_V2::system_clock::time_point _launch_time;
        bool _traj_complete;
        float _time_diff;
        bool _target_reached;
        bool _trajectory_done_prev;
        bool _trajectory_done;
        bool _is_alive;
        float _param_timeout;
    };
}
