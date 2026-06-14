#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"

namespace navigation {

class ComputeTrajectoryBetweenDetections : public BT::StatefulActionNode {
public:
    ComputeTrajectoryBetweenDetections(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    ~ComputeTrajectoryBetweenDetections() override = default;

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<AiDetection>("DetectionA"),
            BT::InputPort<AiDetection>("DetectionB"),
            BT::InputPort<float>("PositionX", 10.0f, "Forward distance to travel after centering between the two detections"),
            BT::BidirectionalPort<Trajectory>("Trajectory"),
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;

private:
    std::shared_ptr<rclcpp::Node> _ros_node;
    rclcpp::Subscription<sonia_common_ros2::msg::Pose>::SharedPtr _pose_sub;
    std::optional<sonia_common_ros2::msg::Pose> _pose_msg;
    std::chrono::_V2::system_clock::time_point _launch_time;
    void pose_call_back(const sonia_common_ros2::msg::Pose& msg);

    AiDetection _det_a, _det_b;
};

}  // namespace navigation
