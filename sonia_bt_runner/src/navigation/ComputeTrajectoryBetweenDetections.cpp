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

    // THe B detection is the shortest detection
    float distanceA, distanceB;
    if (detA.distance > detB.distance) {
        distanceA = detA.distance;
        distanceB = detB.distance;
    } else {
        distanceA = detB.distance;
        distanceB = detA.distance;
    }
    float delta_alpha = std::abs(detA.angle_alpha - detB.angle_alpha) ;

    RCLCPP_INFO(_ros_node->get_logger(),
                "ComputeTrajectoryBetweenDetections: distanceA = %f, distanceB = %f, delta_alpha = %f",
                distanceA, distanceB, delta_alpha);

    float half_cen_dist_betw_det {std::sqrt(distanceA*distanceA + distanceB*distanceB - 2.0f*distanceA*distanceB*std::cos(delta_alpha*(float)M_PI/180.0f))/2.0f};
    float opposite_b_angle {std::asin((distanceB*std::sin(delta_alpha*(float)M_PI/180.0f))/(2.0f*half_cen_dist_betw_det))*180.0f/(float)M_PI};
    // d
    float cen_slalom_dist{std::sqrt(half_cen_dist_betw_det*half_cen_dist_betw_det + distanceA*distanceA - 2.0f*half_cen_dist_betw_det*distanceA*std::cos(opposite_b_angle*180.0f/(float)M_PI))};
    
    float mid_center_det_angle{std::asin(half_cen_dist_betw_det*std::sin(opposite_b_angle*(float)M_PI/180.0f)*180.0f/(float)M_PI)};
    float det_cen_a_angle{180.0f-mid_center_det_angle-opposite_b_angle};
    float rot_to_cen_angle, slalom_orientation;
    if (getInput<std::string>("Side").value() == "Right") {
        rot_to_cen_angle = -(det_cen_a_angle-(delta_alpha - 10.0f));
        slalom_orientation = -mid_center_det_angle+90.0f;
    } else {
        rot_to_cen_angle = det_cen_a_angle-(delta_alpha - 10.0f);
        slalom_orientation = mid_center_det_angle-90.0f;
    }

    // Pose 1: orient the sub to face the midpoint between the two detections.
    TrajectoryPose p0{};
    p0.orientationZ = rot_to_cen_angle;
    p0.frame = 1;

    // Pose 2: drift laterally to the midpoint between the two detections.
    TrajectoryPose p1{};
    p1.positionX = cen_slalom_dist;
    p1.frame = 1;

    // Pose 2: drift laterally to the midpoint between the two detections.
    TrajectoryPose p2{};
    p2.orientationZ = slalom_orientation;
    p2.frame = 1;

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
