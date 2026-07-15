#include "sonia_bt_runner/navigation/ComputeSlalomGateTrajectory.hpp"

#include <algorithm>
#include <cmath>

#define RAD_TO_DEG (180.0f/M_PI)

// A gate is a red pipe between two white pipes, the white pipes can't be
// closer or further apart than these bounds
#define MIN_GATE_WIDTH 1.0f
#define MAX_GATE_WIDTH 10.0f

namespace navigation {

namespace {
struct Cluster
{
    float x;
    float y;
    int count;

    float range() const { return std::hypot(x, y); }
};

float distance_between(const Cluster &cluster, const SlalomPoint &point)
{
    return std::hypot(point.x - cluster.x, point.y - cluster.y);
}
}

ComputeSlalomGateTrajectory::ComputeSlalomGateTrajectory(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), _ros_node(node)
{}

BT::NodeStatus ComputeSlalomGateTrajectory::tick()
{
    COMPUTE_SLALOM_GATE_TRAJECTORY_POINTS_TYPE points;
    if (!getInput<COMPUTE_SLALOM_GATE_TRAJECTORY_POINTS_TYPE>(COMPUTE_SLALOM_GATE_TRAJECTORY_POINTS, points)) {
        RCLCPP_ERROR(_ros_node->get_logger(), "ComputeSlalomGateTrajectory: missing Points");
        return BT::NodeStatus::FAILURE;
    }

    auto side = getInput<COMPUTE_SLALOM_GATE_TRAJECTORY_SIDE_TYPE>(COMPUTE_SLALOM_GATE_TRAJECTORY_SIDE).value();
    if (side != "Left" && side != "Right") {
        RCLCPP_ERROR(_ros_node->get_logger(), "ComputeSlalomGateTrajectory: Side must be Left or Right, got %s", side.c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput<COMPUTE_SLALOM_GATE_TRAJECTORY_TRAJECTORY_TYPE>(COMPUTE_SLALOM_GATE_TRAJECTORY_TRAJECTORY)) {
        RCLCPP_ERROR(_ros_node->get_logger(), "ComputeSlalomGateTrajectory: Trajectory is not initialized");
        return BT::NodeStatus::FAILURE;
    }

    auto cluster_tolerance = getInput<COMPUTE_SLALOM_GATE_TRAJECTORY_CLUSTER_TOLERANCE_TYPE>(COMPUTE_SLALOM_GATE_TRAJECTORY_CLUSTER_TOLERANCE).value();
    auto min_points_per_cluster = getInput<COMPUTE_SLALOM_GATE_TRAJECTORY_MIN_POINTS_PER_CLUSTER_TYPE>(COMPUTE_SLALOM_GATE_TRAJECTORY_MIN_POINTS_PER_CLUSTER).value();
    auto max_gate_range = getInput<COMPUTE_SLALOM_GATE_TRAJECTORY_MAX_GATE_RANGE_TYPE>(COMPUTE_SLALOM_GATE_TRAJECTORY_MAX_GATE_RANGE).value();
    auto approach_standoff = getInput<COMPUTE_SLALOM_GATE_TRAJECTORY_APPROACH_STANDOFF_TYPE>(COMPUTE_SLALOM_GATE_TRAJECTORY_APPROACH_STANDOFF).value();
    auto pass_distance = getInput<COMPUTE_SLALOM_GATE_TRAJECTORY_PASS_DISTANCE_TYPE>(COMPUTE_SLALOM_GATE_TRAJECTORY_PASS_DISTANCE).value();

    RCLCPP_INFO(_ros_node->get_logger(), "ComputeSlalomGateTrajectory: %ld point(s) to cluster", points.points.size());

    // We group the accumulated points into one cluster per pipe, a point joins
    // the closest cluster within the tolerance, otherwise it opens a new one
    std::vector<Cluster> clusters;
    for (const auto &point : points.points) {
        Cluster *closest_cluster = nullptr;
        float closest_dist = cluster_tolerance;
        for (auto &cluster : clusters) {
            float dist = distance_between(cluster, point);
            if (dist <= closest_dist) {
                closest_cluster = &cluster;
                closest_dist = dist;
            }
        }
        if (closest_cluster) {
            // We keep the cluster center on the running average of its points
            closest_cluster->x = (closest_cluster->x * closest_cluster->count + point.x) / (closest_cluster->count + 1);
            closest_cluster->y = (closest_cluster->y * closest_cluster->count + point.y) / (closest_cluster->count + 1);
            closest_cluster->count++;
        } else {
            clusters.push_back({point.x, point.y, 1});
        }
    }

    // We drop the noise clusters and everything outside the expected gate range
    std::vector<Cluster> pipes;
    for (const auto &cluster : clusters) {
        RCLCPP_INFO(_ros_node->get_logger(), "Cluster at x = %f, y = %f with %d point(s), range = %f",
                    cluster.x, cluster.y, cluster.count, cluster.range());
        if (cluster.count >= min_points_per_cluster && cluster.range() <= max_gate_range) {
            pipes.push_back(cluster);
        }
    }

    RCLCPP_INFO(_ros_node->get_logger(), "We keep %ld pipe(s) out of %ld cluster(s)", pipes.size(), clusters.size());

    if (pipes.size() < 3) {
        RCLCPP_ERROR(_ros_node->get_logger(), "We need at least 3 pipes to find the gate, got %ld", pipes.size());
        return BT::NodeStatus::FAILURE;
    }

    // The 3 closest pipes are the current layer, the next layer is behind it
    std::sort(pipes.begin(), pipes.end(), [](const Cluster &a, const Cluster &b) {
        return a.range() < b.range();
    });
    pipes.resize(3);

    // The two pipes the furthest apart are the ends of the gate, the remaining
    // one is the red pipe in the middle
    int end_a = 0, end_b = 1;
    float gate_width = -1.0f;
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 3; j++) {
            float separation = std::hypot(pipes[j].x - pipes[i].x, pipes[j].y - pipes[i].y);
            if (separation > gate_width) {
                gate_width = separation;
                end_a = i;
                end_b = j;
            }
        }
    }
    int middle = 3 - end_a - end_b;

    if (gate_width < MIN_GATE_WIDTH || gate_width > MAX_GATE_WIDTH) {
        RCLCPP_ERROR(_ros_node->get_logger(), "The pipe layout doesn't look like a slalom gate (width = %f)", gate_width);
        return BT::NodeStatus::FAILURE;
    }

    const Cluster &red = pipes[middle];
    RCLCPP_INFO(_ros_node->get_logger(), "Assumed red pipe at x = %f, y = %f", red.x, red.y);

    // We pick the white pipe on the wanted side of the red pipe. A positive
    // rotation goes toward positive y, so a positive relative bearing is on
    // the right of the red pipe
    float red_bearing = std::atan2(red.y, red.x);
    auto relative_bearing = [red_bearing](const Cluster &cluster) {
        float relative = std::atan2(cluster.y, cluster.x) - red_bearing;
        while (relative > M_PI) relative -= 2.0f * M_PI;
        while (relative <= -M_PI) relative += 2.0f * M_PI;
        return relative;
    };

    const Cluster *white;
    if ((relative_bearing(pipes[end_a]) > 0.0f) == (side == "Right")) {
        white = &pipes[end_a];
    } else {
        white = &pipes[end_b];
    }
    RCLCPP_INFO(_ros_node->get_logger(), "Passing on side %s, between the red pipe and the white pipe at x = %f, y = %f",
                side.c_str(), white->x, white->y);

    // We aim the middle of the gap and cross it perpendicularly to the gate
    float gap_x = (red.x + white->x) / 2.0f;
    float gap_y = (red.y + white->y) / 2.0f;

    float gate_dir_x = (pipes[end_b].x - pipes[end_a].x) / gate_width;
    float gate_dir_y = (pipes[end_b].y - pipes[end_a].y) / gate_width;
    float normal_x = -gate_dir_y;
    float normal_y = gate_dir_x;
    // The normal has to point away from the sub so that we cross the gate
    if (normal_x * gap_x + normal_y * gap_y < 0.0f) {
        normal_x = -normal_x;
        normal_y = -normal_y;
    }
    float gate_heading = std::atan2(normal_y, normal_x) * RAD_TO_DEG;

    float approach_x = gap_x - approach_standoff * normal_x;
    float approach_y = gap_y - approach_standoff * normal_y;
    float approach_dist = std::hypot(approach_x, approach_y);

    RCLCPP_INFO(_ros_node->get_logger(), "Gap at x = %f, y = %f, approach point at x = %f, y = %f, gate heading = %f",
                gap_x, gap_y, approach_x, approach_y, gate_heading);

    auto traj = getInput<COMPUTE_SLALOM_GATE_TRAJECTORY_TRAJECTORY_TYPE>(COMPUTE_SLALOM_GATE_TRAJECTORY_TRAJECTORY).value();

    if (approach_dist > 0.1f) {
        // Pose 1: rotate the sub to aim the approach point
        TrajectoryPose p0{};
        p0.orientationZ = std::atan2(approach_y, approach_x) * RAD_TO_DEG;
        p0.frame = 2;
        traj.trajectory.push_back(p0);

        // Pose 2: move forward to the approach point
        TrajectoryPose p1{};
        p1.positionX = approach_dist;
        p1.frame = 1;
        traj.trajectory.push_back(p1);
    }

    // Pose 3: rotate the sub to be perpendicular to the gate
    TrajectoryPose p2{};
    p2.orientationZ = gate_heading;
    p2.frame = 2;
    traj.trajectory.push_back(p2);

    // Pose 4: move forward through the gap
    TrajectoryPose p3{};
    p3.positionX = approach_standoff + pass_distance;
    p3.frame = 1;
    traj.trajectory.push_back(p3);

    setOutput<COMPUTE_SLALOM_GATE_TRAJECTORY_TRAJECTORY_TYPE>(COMPUTE_SLALOM_GATE_TRAJECTORY_TRAJECTORY, traj);
    setOutput(COMPUTE_SLALOM_GATE_TRAJECTORY_GATE_HEADING, gate_heading);

    return BT::NodeStatus::SUCCESS;
}

}  // namespace navigation
