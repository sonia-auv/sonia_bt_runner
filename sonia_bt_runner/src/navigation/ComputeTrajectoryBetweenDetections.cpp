#include "sonia_bt_runner/navigation/ComputeTrajectoryBetweenDetections.hpp"

namespace navigation {

ComputeTrajectoryBetweenDetections::ComputeTrajectoryBetweenDetections(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), _ros_node(node)
{}

BT::NodeStatus ComputeTrajectoryBetweenDetections::tick()
{
    if (!getInput<AiDetection>("DetectionA", _det_a) || !getInput<AiDetection>("DetectionB", _det_b)) {
        RCLCPP_ERROR(_ros_node->get_logger(), "ComputeTrajectoryBetweenDetections: missing DetectionA or DetectionB");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput<Trajectory>("Trajectory")) {
        RCLCPP_ERROR(_ros_node->get_logger(), "ComputeTrajectoryBetweenDetections: Trajectory is not initialized");
        return BT::NodeStatus::FAILURE;
    }

    float a_det_dist{_det_a.distance};
    float b_det_dist{_det_b.distance};
    float delta_alpha{std::abs(_det_a.angle_alpha - _det_a.angle_alpha)};
    float cen_dist_betw_det{std::sqrt(a_det_dist*a_det_dist + b_det_dist*b_det_dist - 2.0f*a_det_dist*b_det_dist*std::cos(delta_alpha))/2.0f};
    float det_b_angle, det_a_angle;
    if (a_det_dist > b_det_dist) {
        det_a_angle = std::asin((b_det_dist*std::sin(delta_alpha))/(2.0f*cen_dist_betw_det));
    } else {
        det_b_angle = std::asin((a_det_dist*std::sin(delta_alpha))/(2.0f*cen_dist_betw_det));
        det_a_angle = 180.0f-delta_alpha-det_b_angle;
    }
    float cen_slalom_dist{std::sqrt(cen_dist_betw_det*cen_dist_betw_det + a_det_dist*a_det_dist - 2.0f*cen_dist_betw_det*a_det_dist*std::cos(det_a_angle))};
    float plane_slalom_angle{std::asin(a_det_dist*std::sin(det_a_angle)/cen_slalom_dist)};
    float det_cen_a_angle{180.0f-plane_slalom_angle-det_a_angle};
    float rot_to_cen_angle{a_det_dist - det_cen_a_angle};
    float slalom_orientation{90.0f-plane_slalom_angle};

    // float positionX = getInput<float>("PositionX").value_or(10.0f);
    // float midY = (detA.distance_teta + detB.distance_teta) / 2.0f;
    // float midAngle = (detA.angle_alpha + detB.angle_alpha) / 2.0f;

    // RCLCPP_INFO(_ros_node->get_logger(),
    //             "ComputeTrajectoryBetweenDetections: midAngle=%.2f midY=%.2f positionX=%.2f",
    //             midAngle, midY, positionX);

    // Pose 1: orient the sub to face the midpoint between the two detections.
    TrajectoryPose p0{};
    p0.orientationZ = rot_to_cen_angle;
    p0.frame = 1;

    // Pose 2: drift laterally to the midpoint between the two detections.
    TrajectoryPose p1{};
    p1.positionX = cen_slalom_dist;
    p1.frame = 1;

    // Pose 3: move forward through the gap.
    TrajectoryPose p2{};
    p2.orientationZ = slalom_orientation;
    p2.frame = 1;

    TrajectoryPose p3{};
    p2.positionX = getInput<float>("PositionX").value();
    p2.frame = 1;

    Trajectory traj = getInput<Trajectory>("Trajectory").value();
    traj.trajectory.push_back(p0);
    traj.trajectory.push_back(p1);
    traj.trajectory.push_back(p2);
    traj.trajectory.push_back(p3);
    setOutput<Trajectory>("Trajectory", traj);

    return BT::NodeStatus::SUCCESS;
}

}  // namespace navigation
