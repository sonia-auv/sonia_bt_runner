#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "std_msgs/msg/int8.hpp"
#include "sonia_common_ros2/msg/pose.hpp"
#include "sonia_common_ros2/msg/pose_array.hpp"
#include "sonia_common_ros2/msg/mpc_info.hpp"
#include "sonia_bt_runner/utils/AiDetection.hpp"

namespace navigation {

class MoveBetweenDetections : public BT::StatefulActionNode {
public:
    MoveBetweenDetections(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    ~MoveBetweenDetections() override = default;

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<AiDetection>("DetectionA"),
            BT::InputPort<AiDetection>("DetectionB"),
            BT::InputPort<float>("PositionX", 10.0f, "Forward distance to travel after centering between the two detections"),
            BT::InputPort<float>("TimeBeforeFailing", 15.0f, "Timeout in seconds before the movement is considered failed"),
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void planner_callback(const std_msgs::msg::Int8 &msg);
    void controller_callback(const sonia_common_ros2::msg::MpcInfo &msg);

    enum class Phase { WAITING_PLANNER, WAITING_TARGET };

    std::shared_ptr<rclcpp::Node> _ros_node;
    rclcpp::Publisher<sonia_common_ros2::msg::PoseArray>::SharedPtr _planner_pub;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr _planner_sub;
    rclcpp::Subscription<sonia_common_ros2::msg::MpcInfo>::SharedPtr _controller_sub;

    Phase _phase;
    int _planner_valid;
    bool _target_reached;
    bool _trajectory_done;
    bool _trajectory_done_prev;
    bool _mpc_alive;
    std::chrono::_V2::system_clock::time_point _launch_time;
    float _timeout;
};

}  // namespace navigation
