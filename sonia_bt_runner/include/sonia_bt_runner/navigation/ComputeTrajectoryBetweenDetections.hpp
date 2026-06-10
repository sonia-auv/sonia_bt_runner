#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"

namespace navigation {

class ComputeTrajectoryBetweenDetections : public BT::SyncActionNode {
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

    BT::NodeStatus tick() override;

private:
    std::shared_ptr<rclcpp::Node> _ros_node;
};

}  // namespace navigation
