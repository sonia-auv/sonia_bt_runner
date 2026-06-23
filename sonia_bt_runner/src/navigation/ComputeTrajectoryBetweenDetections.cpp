#include "sonia_bt_runner/navigation/ComputeTrajectoryBetweenDetections.hpp"

#define RAD_TO_DEG 180.0f/M_PI
#define DEG_TO_RAD M_PI/180.0f

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
    float rot_angle_to_aim_betw_two_det{(furthest_det_angle + nearest_det_angle)/(-2.0f)};

    // We compute the angle between the two detection angle
    float delta_alpha{std::abs(furthest_det_angle - nearest_det_angle)};

    // We compute the half of the distance between the 2 detection
    float half_dist_betw_det{std::sqrt(furthest_det_dist*furthest_det_dist + nearest_det_dist*nearest_det_dist 
                                       - 2.0f*furthest_det_dist*nearest_det_dist*std::cos(
                                           delta_alpha
                                       ))/2.0f};

    // We compute the angle made on the furtest detection between the sub and the nearest detection
    float furthest_det_angle{std::asin((furthest_det_dist*std::sin(delta_alpha))/(2.0f*half_dist_betw_det))};
    
    // We compute the travaling distance that the sub need to do to go between the two detection
    float moving_dist{std::sqrt(half_dist_betw_det*half_dist_betw_det + nearest_det_dist*nearest_det_dist
                                - 2.0f*half_dist_betw_det*nearest_det_dist*std::cos(furthest_det_angle))};
    
    // We compute the angle between the furtest detection and the direction that cross between the 2 detection
    float furthest_sub_angle {std::asin((half_dist_betw_det*std::sin(furthest_det_angle))/moving_dist)};
    
    // We compute the angle that the sub need to do to be perpendicular with the slalom
    float angle_to_be_perp {M_PI_2-furthest_det_angle-furthest_sub_angle};

    // Pose 1: rotate the sub to aim it at half distance 
    TrajectoryPose p0{};
    p0.orientationZ = rot_angle_to_aim_betw_two_det*RAD_TO_DEG;
    p0.frame = 1;

    // Pose 2: Move forward to be between the 2 detections
    TrajectoryPose p1{};
    p1.positionX = moving_dist;
    p1.frame = 1;

    // Pose 2: We rotate the sub to be perpendicar with the two detection.
    TrajectoryPose p2{};
    p2.orientationZ = angle_to_be_perp*RAD_TO_DEG;
    p2.frame = 1;

    if (nearest_det_angle < furthest_det_angle) {
        p2.orientationZ *= -1.0f;
    }

    float positionX = getInput<float>("PositionX").value_or(10.0f);
    // Pose 3: move forward through the gap.
    TrajectoryPose p3{};
    p3.positionX = positionX;
    p3.frame = 1;

    Trajectory traj = getInput<Trajectory>("Trajectory").value();
    traj.trajectory.push_back(p0);
    traj.trajectory.push_back(p1);
    traj.trajectory.push_back(p2);
    traj.trajectory.push_back(p3);
    setOutput<Trajectory>("Trajectory", traj);

    return BT::NodeStatus::SUCCESS;
}

}  // namespace navigation
