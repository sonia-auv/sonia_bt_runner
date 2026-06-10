#include "sonia_bt_runner/navigation/ComputeTrajectoryBetweenDetections.hpp"

namespace navigation {

ComputeTrajectoryBetweenDetections::ComputeTrajectoryBetweenDetections(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), _ros_node(node)
{}

BT::NodeStatus ComputeTrajectoryBetweenDetections::tick()
{
    AiDetection detA, detB;
    if (!getInput<AiDetection>("DetectionA", detA) || !getInput<AiDetection>("DetectionB", detB)) {
        RCLCPP_ERROR(_ros_node->get_logger(), "ComputeTrajectoryBetweenDetections: missing DetectionA or DetectionB");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput<Trajectory>("Trajectory")) {
        RCLCPP_ERROR(_ros_node->get_logger(), "ComputeTrajectoryBetweenDetections: Trajectory is not initialized");
        return BT::NodeStatus::FAILURE;
    }

    float positionX = getInput<float>("PositionX").value_or(10.0f);
    float midY = (detA.distance_teta + detB.distance_teta) / 2.0f;
    float midAngle = (detA.angle_alpha + detB.angle_alpha) / 2.0f;

    RCLCPP_INFO(_ros_node->get_logger(),
                "ComputeTrajectoryBetweenDetections: midAngle=%.2f midY=%.2f positionX=%.2f",
                midAngle, midY, positionX);

    // Pose 1: orient the sub to face the midpoint between the two detections.
    TrajectoryPose p0{};
    p0.orientationZ = -midAngle;
    p0.frame = 1;

    // Pose 2: drift laterally to the midpoint between the two detections.
    TrajectoryPose p1{};
    p1.positionY = midY;
    p1.frame = 1;

    // Pose 3: move forward through the gap.
    TrajectoryPose p2{};
    p2.positionX = positionX;
    p2.frame = 1;

    Trajectory traj = getInput<Trajectory>("Trajectory").value();
    traj.trajectory.push_back(p0);
    traj.trajectory.push_back(p1);
    traj.trajectory.push_back(p2);
    setOutput<Trajectory>("Trajectory", traj);

    return BT::NodeStatus::SUCCESS;
}

}  // namespace navigation
