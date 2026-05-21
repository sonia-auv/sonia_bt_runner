#include "sonia_bt_runner/navigation/TorpedoAiming.hpp"

namespace navigation
{

    TorpedoAiming::TorpedoAiming(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), _ros_node(node)
    {

    }

    BT::NodeStatus TorpedoAiming::tick()
    {
        AiDetection det;
        std::string launching_side;
        float target_offset;

        // We get the detected object by the AI
        getInput("Detection", det);
        getInput("LaunchingSide", launching_side);
        getInput("TargetOffset", target_offset);

        if(!getInput<Trajectory>("Trajectory"))
        {
            RCLCPP_INFO(_ros_node->get_logger(), "The Trajectory is not initialize");

            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(_ros_node->get_logger(), "Computing TorpedoAiming trajectory");
        
        TrajectoryPose t;
        t.positionX = det.distance - target_offset; // We take an offset with the target poster
        t.positionY = det.distance_teta; // We move on the y axis
        if (launching_side == "port_side")
        {
            t.positionY -= CAMERA_TO_TORPIDO_PEPPER_OFFSET_X; // We move on the x axis
        }
        else if(launching_side == "starboard")
        {
            t.positionY -= CAMERA_TO_TORPIDO_SALT_OFFSET_X; // We move on the x axis
        }
        t.positionZ = det.distance_beta; // We move on the z axis
        if (launching_side == "port_side" || launching_side == "starboard")
        {
            t.positionZ -= CAMERA_TO_TORPEDO_OFFSET_Y;
        }
        t.orientationX = 0.0;       // We don't rotate on the x axis
        t.orientationY = 0.0;       // We don't rotate on the y axis
        t.orientationZ = 0.0;       // We don't rotate on the z axis
        t.frame = 1;                // We use the relative positionning
        t.speed = 0;
        t.precision = 0;            // We don't care about the precision
        t.long_rotation = false;    // We don't care about the long rotation

        RCLCPP_INFO(_ros_node->get_logger(), "position en x: %f", t.positionX);
        RCLCPP_INFO(_ros_node->get_logger(), "position en y: %f", t.positionY);
        RCLCPP_INFO(_ros_node->get_logger(), "position en z: %f", t.positionZ);
        RCLCPP_INFO(_ros_node->get_logger(), "orientation en x: %f", t.orientationX);
        RCLCPP_INFO(_ros_node->get_logger(), "orientation en y: %f", t.orientationY);
        RCLCPP_INFO(_ros_node->get_logger(), "orientation en z: %f", t.orientationZ);

        // We append the new pose to the trajectory
        Trajectory traj = getInput<Trajectory>("Trajectory").value();
        traj.trajectory.push_back(t);
        setOutput<Trajectory>("Trajectory", traj);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation