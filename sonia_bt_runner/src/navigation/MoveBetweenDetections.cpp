#include "sonia_bt_runner/navigation/MoveBetweenDetections.hpp"

using std::placeholders::_1;

namespace navigation {

MoveBetweenDetections::MoveBetweenDetections(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), _ros_node(node)
{
    _planner_pub = _ros_node->create_publisher<sonia_common_ros2::msg::PoseArray>("/proc_planner/send_pose_array", 10);
}

BT::NodeStatus MoveBetweenDetections::onStart()
{
    AiDetection detA, detB;
    if (!getInput<AiDetection>("DetectionA", detA) || !getInput<AiDetection>("DetectionB", detB)) {
        RCLCPP_ERROR(_ros_node->get_logger(), "MoveBetweenDetections: missing DetectionA or DetectionB");
        return BT::NodeStatus::FAILURE;
    }

    float positionX = getInput<float>("PositionX").value_or(10.0f);
    _timeout = getInput<float>("TimeBeforeFailing").value_or(15.0f);

    float midY = (detA.distance_teta + detB.distance_teta) / 2.0f;

    RCLCPP_INFO(_ros_node->get_logger(),
                "MoveBetweenDetections: detA.distance_teta=%.2f detB.distance_teta=%.2f midY=%.2f posX=%.2f",
                detA.distance_teta, detB.distance_teta, midY, positionX);

    // Build a two-pose trajectory: first center laterally (between two detections), then go forward.
    sonia_common_ros2::msg::PoseArray array;
    array.interpolation_method = 0;

    // Pose 1 — lateral centering between the two slaloms.
    sonia_common_ros2::msg::Pose p1;
    p1.position.x = 0.0f;
    p1.position.y = midY;
    p1.position.z = 0.0f;
    p1.orientation.x = 0.0f;
    p1.orientation.y = 0.0f;
    p1.orientation.z = 0.0f;
    p1.frame = 1;
    p1.speed = 0;
    p1.fine = 0;
    p1.rotation = false;
    array.poses.push_back(p1);

    // Pose 2 — forward move through the two objects.
    sonia_common_ros2::msg::Pose p2;
    p2.position.x = positionX;
    p2.position.y = 0.0f;
    p2.position.z = 0.0f;
    p2.orientation.x = 0.0f;
    p2.orientation.y = 0.0f;
    p2.orientation.z = 0.0f;
    p2.frame = 1;
    p2.speed = 0;
    p2.fine = 0;
    p2.rotation = false;

    array.poses.push_back(p2);

    _planner_pub->publish(array);

    _planner_valid = 1;
    _target_reached = false;
    _trajectory_done = false;
    _trajectory_done_prev = false;
    _mpc_alive = true;
    _phase = Phase::WAITING_PLANNER;
    _launch_time = std::chrono::system_clock::now();

    _planner_sub = _ros_node->create_subscription<std_msgs::msg::Int8>(
        "/proc_planner/is_waypoints_valid", 10,
        std::bind(&MoveBetweenDetections::planner_callback, this, _1));

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveBetweenDetections::onRunning()
{
    std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - _launch_time;

    if (elapsed.count() > _timeout) {
        RCLCPP_WARN(_ros_node->get_logger(), "MoveBetweenDetections: timed out after %.1fs", _timeout);
        _planner_sub.reset();
        _controller_sub.reset();
        return BT::NodeStatus::FAILURE;
    }

    if (!_mpc_alive) {
        RCLCPP_ERROR(_ros_node->get_logger(), "MoveBetweenDetections: MPC is not alive");
        _controller_sub.reset();
        return BT::NodeStatus::FAILURE;
    }

    if (_phase == Phase::WAITING_PLANNER) {
        // Wait for the planner to accept and start executing our trajectory.
        if (_planner_valid == 0) {
            _planner_sub.reset();
            _phase = Phase::WAITING_TARGET;
            _controller_sub = _ros_node->create_subscription<sonia_common_ros2::msg::MpcInfo>(
                "/proc_control/controller_info", 1,
                std::bind(&MoveBetweenDetections::controller_callback, this, _1));
            RCLCPP_INFO(_ros_node->get_logger(), "MoveBetweenDetections: planner validated, waiting for target");
        }
        return BT::NodeStatus::RUNNING;
    }

    if (_target_reached) {
        RCLCPP_INFO(_ros_node->get_logger(), "MoveBetweenDetections: target reached");
        _controller_sub.reset();
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void MoveBetweenDetections::onHalted()
{
    _planner_sub.reset();
    _controller_sub.reset();
}

void MoveBetweenDetections::planner_callback(const std_msgs::msg::Int8 &msg)
{
    _planner_valid = msg.data;
}

void MoveBetweenDetections::controller_callback(const sonia_common_ros2::msg::MpcInfo &msg)
{
    _mpc_alive = msg.is_mpc_alive;
    _target_reached = msg.target_reached;
    _trajectory_done = msg.is_trajectory_done;

    if (_trajectory_done != _trajectory_done_prev) {
        if (_trajectory_done)
            RCLCPP_INFO(_ros_node->get_logger(), "MoveBetweenDetections: trajectory completed");
        else
            RCLCPP_INFO(_ros_node->get_logger(), "MoveBetweenDetections: trajectory received by controller");
    }
    _trajectory_done_prev = _trajectory_done;
}

}  // namespace navigation
