#include "sonia_bt_runner/navigation/ComputeTrajectoryBetweenDetections.hpp"

#define RAD_TO_DEG (180.0f/M_PI)
#define DEG_TO_RAD (M_PI/180.0f)

namespace navigation {

ComputeTrajectoryBetweenDetections::ComputeTrajectoryBetweenDetections(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), _ros_node(node)
{}

BT::NodeStatus ComputeTrajectoryBetweenDetections::tick()
{
    COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_DETECTION_A_TYPE detA;
    COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_DETECTION_B_TYPE detB;
    if (!getInput<COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_DETECTION_A_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_DETECTION_A, detA) ||
        !getInput<COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_DETECTION_B_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_DETECTION_B, detB)) {
        RCLCPP_ERROR(_ros_node->get_logger(), "ComputeTrajectoryBetweenDetections: missing DetectionA or DetectionB");
        return BT::NodeStatus::FAILURE;
    }

    auto angle = getInput<COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_DETECTION_ANGLE_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_DETECTION_ANGLE).value();
    auto offset_angle = getInput<COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_OFFSET_ANGLE_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_OFFSET_ANGLE).value();
    angle *= DEG_TO_RAD;
    if (!getInput<COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_TRAJECTORY_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_TRAJECTORY)) {
        RCLCPP_ERROR(_ros_node->get_logger(), "ComputeTrajectoryBetweenDetections: Trajectory is not initialized");
        return BT::NodeStatus::FAILURE;
    }

    [[maybe_unused]] auto side = getInput<COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_SIDE_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_SIDE).value();

    // We put the detection angle in good named variable
    float nearest_det_dist, furthest_det_dist, nearest_det_angle, furthest_det_angle;
    if (detA.distance > detB.distance) {
        furthest_det_dist = detA.distance;
        furthest_det_angle = DEG_TO_RAD*detA.angle_alpha;
        nearest_det_dist = detB.distance;
        nearest_det_angle = DEG_TO_RAD*detB.angle_alpha;
    } else {
        furthest_det_dist = detB.distance;
        furthest_det_angle = DEG_TO_RAD*detB.angle_alpha;
        nearest_det_dist = detA.distance;
        nearest_det_angle = DEG_TO_RAD*detA.angle_alpha;
    }
    
    // We compute the rotation angle to aim the sub at mid distance between the two detections
    // float rot_angle_to_aim_betw_two_det{(furthest_det_angle + nearest_det_angle - std::abs(angle))/(-2.0f)};

    // We compute the angle between the two detection angle
    float delta_alpha{std::abs(furthest_det_angle - nearest_det_angle) + std::abs(angle)};
    RCLCPP_INFO(_ros_node->get_logger(), "delta_alpha : %lf", delta_alpha*RAD_TO_DEG);

    // We compute the half of the distance between the 2 detection
    float half_dist_betw_det{std::sqrt(furthest_det_dist * furthest_det_dist + nearest_det_dist * nearest_det_dist
                                       - 2.0f * furthest_det_dist * nearest_det_dist * std::cos(
                                           delta_alpha
                                       )) / 2.0f};
    RCLCPP_INFO(_ros_node->get_logger(), "half_dist_betw_det : %lf", half_dist_betw_det);

    // We compute the angle made on the furtest detection between the sub and the nearest detection
    float furthest_angle{std::asin((nearest_det_dist * std::sin(delta_alpha))/(2.0f * half_dist_betw_det))};
    RCLCPP_INFO(_ros_node->get_logger(), "furthest_angle : %lf", furthest_angle*RAD_TO_DEG);
    
    // We compute the travaling distance that the sub need to do to go between the two detection
    float moving_dist{std::sqrt(half_dist_betw_det * half_dist_betw_det + furthest_det_dist * furthest_det_dist
                                - 2.0f * half_dist_betw_det * furthest_det_dist * std::cos(furthest_angle))};
    RCLCPP_INFO(_ros_node->get_logger(), "moving_dist : %lf", moving_dist);
    
    // We compute the angle between the furtest detection and the direction that cross between the 2 detection
    float furthest_sub_angle {std::asin((half_dist_betw_det*std::sin(furthest_angle))/moving_dist)};
    RCLCPP_INFO(_ros_node->get_logger(), "furthest_sub_angle : %lf", furthest_sub_angle*RAD_TO_DEG);
    
    // We compute the angle that the sub need to do to be perpendicular with the slalom
    float angle_to_be_perp {(float)M_PI_2-furthest_angle-furthest_sub_angle};
    RCLCPP_INFO(_ros_node->get_logger(), "angle_to_be_perp : %lf", angle_to_be_perp*RAD_TO_DEG);

    // Pose 1: rotate the sub to aim it at half distance 
    TrajectoryPose p0{};
	p0.orientationZ = - offset_angle + (RAD_TO_DEG * angle / 2.0f) - (detA.angle_alpha / 2.0f);
    RCLCPP_INFO(_ros_node->get_logger(), "Angle to aim the center of the slalom : %lf", p0.orientationZ);
    RCLCPP_INFO(_ros_node->get_logger(), "OffsetAngle : %lf", offset_angle);
    RCLCPP_INFO(_ros_node->get_logger(), "Angle : %lf", angle);
    RCLCPP_INFO(_ros_node->get_logger(), "The alpha angle of the closest white slalom : %lf", detA.angle_alpha * RAD_TO_DEG);

    p0.frame = 1;

    // Pose 2: Move forward to be between the 2 detections
    TrajectoryPose p1{};
    p1.positionX = moving_dist;
    p1.frame = 1;

    // Pose 2: We rotate the sub to be perpendicar with the two detection.
    TrajectoryPose p2{};
    p2.orientationZ = angle_to_be_perp*RAD_TO_DEG;

    p2.frame = 1;

    if (nearest_det_angle > furthest_det_angle) {
        p2.orientationZ *= -1.0f;
    }

    // Pose 3: turn to make the sub streight
    // TrajectoryPose p3{};
    // p3.orientationZ = 0;
    // p3.frame = 2;

    auto positionX = getInput<COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_POSITION_X_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_POSITION_X).value_or(10.0f);
    // Pose 4: move forward through the gap.
    TrajectoryPose p4{};
    p4.positionX = positionX;
    p4.frame = 1;

    auto traj = getInput<COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_TRAJECTORY_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_TRAJECTORY).value();
    traj.trajectory.push_back(p0);
    traj.trajectory.push_back(p1);
    traj.trajectory.push_back(p2);
    // traj.trajectory.push_back(p3);
    traj.trajectory.push_back(p4);
    setOutput<COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_TRAJECTORY_TYPE>(COMPUTE_TRAJECTORY_BETWEEN_DETECTIONS_TRAJECTORY, traj);

    return BT::NodeStatus::SUCCESS;
}

}  // namespace navigation
