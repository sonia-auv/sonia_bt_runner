#include "sonia_bt_runner/navigation/MoveAboveObject.hpp"

namespace navigation
{

    MoveAboveObject::MoveAboveObject(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), _ros_node(node)
    {
        
    }

    BT::NodeStatus MoveAboveObject::tick()
    {
        AiDetection det;

        // We get the detected object by the AI
        getInput("Detection", det);

        if(!getInput<Trajectory>("Trajectory"))
        {
            RCLCPP_INFO(_ros_node->get_logger(), "The Trajectory is not initialize");

            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(_ros_node->get_logger(), "Computing MoveAboveObject trajectory");
        
        // We compute the trajectory to do
        TrajectoryPose t;

        // Switch dropper if multiple dropper. Need an input for that
        t.positionX = -det.distance_beta - CAMERA_TO_DROPPER_OFFSET_X;
        t.positionY = det.distance_teta - CAMERA_TO_DROPPER_OFFSET_Y;
        t.positionZ = 0.0;
        t.orientationX = 0.0;
        t.orientationY = 0.0;
        t.orientationZ = 0.0;
        t.frame = 1;
        t.speed = 0;
        t.precision = 0;
        t.long_rotation = false;

        RCLCPP_INFO(_ros_node->get_logger(), "position en x: %f", t.positionX);
        RCLCPP_INFO(_ros_node->get_logger(), "position en y: %f", t.positionY);
        RCLCPP_INFO(_ros_node->get_logger(), "position en z: %f", t.positionZ);
        RCLCPP_INFO(_ros_node->get_logger(), "orientation en x: %f", t.orientationX);
        RCLCPP_INFO(_ros_node->get_logger(), "orientation en y: %f", t.orientationY);
        RCLCPP_INFO(_ros_node->get_logger(), "orientation en z: %f", t.orientationZ);

        // We append the new trajectory point to the sending trajectory
        Trajectory traj = getInput<Trajectory>("Trajectory").value();
        traj.trajectory.push_back(t);
        setOutput<Trajectory>("Trajectory", traj);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation
