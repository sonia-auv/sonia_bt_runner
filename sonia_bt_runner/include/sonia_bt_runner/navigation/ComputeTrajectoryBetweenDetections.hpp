#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"

#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_A_PARAM "DetectionA"
#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_A_TYPE AiDetection
#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_B_PARAM "DetectionB"
#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_B_TYPE AiDetection
#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_POSITION_X_PARAM "PositionX"
#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_POSITION_X_TYPE float
#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_TRAJECTORY_PARAM "Trajectory"
#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_TRAJECTORY_TYPE Trajectory

namespace navigation {

class ComputeTrajectoryBetweenDetections : public BT::SyncActionNode {
public:
    ComputeTrajectoryBetweenDetections(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    ~ComputeTrajectoryBetweenDetections() override = default;

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_A_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_A_PARAM),
            BT::InputPort<COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_B_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_B_PARAM),
            BT::InputPort<COMPUTE_TRAJECTORY_BETWEEN_DETECTION_POSITION_X_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTION_POSITION_X_PARAM, 10.0f, "Forward distance to travel after centering between the two detections"),
            BT::BidirectionalPort<COMPUTE_TRAJECTORY_BETWEEN_DETECTION_TRAJECTORY_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTION_TRAJECTORY_PARAM),
        };
    }

    BT::NodeStatus tick() override;

private:
    std::shared_ptr<rclcpp::Node> _ros_node;
    COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_A_TYPE _det_a;
    COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_B_TYPE _det_b;
};

}  // namespace navigation
