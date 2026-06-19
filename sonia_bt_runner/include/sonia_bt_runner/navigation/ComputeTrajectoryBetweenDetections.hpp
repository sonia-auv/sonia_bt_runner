#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"

#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_A_PARAM_NAME "DetectionA"
#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_A_PARAM_TYPE AiDetection
#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_B_PARAM_NAME "DetectionB"
#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_B_PARAM_TYPE AiDetection
#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_POSITION_X_PARAM_NAME "PositionX"
#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_POSITION_X_PARAM_TYPE float
#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_TRAJECTORY_PARAM_NAME "Trajectory"
#define COMPUTE_TRAJECTORY_BETWEEN_DETECTION_TRAJECTORY_PARAM_TYPE Trajectory

namespace navigation {

class ComputeTrajectoryBetweenDetections : public BT::SyncActionNode {
public:
    ComputeTrajectoryBetweenDetections(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    ~ComputeTrajectoryBetweenDetections() override = default;

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_A_PARAM_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_A_PARAM_NAME),
            BT::InputPort<COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_B_PARAM_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_B_PARAM_NAME),
            BT::InputPort<COMPUTE_TRAJECTORY_BETWEEN_DETECTION_POSITION_X_PARAM_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTION_POSITION_X_PARAM_NAME, 10.0f, "Forward distance to travel after centering between the two detections"),
            BT::BidirectionalPort<COMPUTE_TRAJECTORY_BETWEEN_DETECTION_TRAJECTORY_PARAM_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTION_TRAJECTORY_PARAM_NAME),
        };
    }

    BT::NodeStatus tick() override;

private:
    std::shared_ptr<rclcpp::Node> _ros_node;
    COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_A_PARAM_TYPE _det_a;
    COMPUTE_TRAJECTORY_BETWEEN_DETECTION_DETECTION_B_PARAM_TYPE _det_b;
};

}  // namespace navigation
