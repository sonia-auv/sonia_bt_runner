#include "sonia_bt_runner/navigation/TorpedoAiming.hpp"

namespace navigation
{

    TorpedoAiming::TorpedoAiming(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), _ros_node(node)
    {

    }

    BT::NodeStatus TorpedoAiming::tick()
    {
        TORPEDO_AIMING_DETECTION_TYPE det;
        TORPEDO_AIMING_LAUNCHING_SIDE_TYPE launching_side;
        TORPEDO_AIMING_TARGET_OFFSET_TYPE target_offset;

        // We get the detected object by the AI
        getInput(TORPEDO_AIMING_DETECTION, det);
        getInput(TORPEDO_AIMING_LAUNCHING_SIDE, launching_side);
        getInput(TORPEDO_AIMING_TARGET_OFFSET, target_offset);

        if(!getInput<TORPEDO_AIMING_TRAJECTORY_TYPE>(TORPEDO_AIMING_TRAJECTORY))
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
        t.positionZ = -det.distance_beta; // We move on the z axis
        if (launching_side == "port_side" || launching_side == "starboard")
        {
            t.positionZ += CAMERA_TO_TORPEDO_OFFSET_Y;
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
        auto traj = getInput<TORPEDO_AIMING_TRAJECTORY_TYPE>(TORPEDO_AIMING_TRAJECTORY).value();
        traj.trajectory.push_back(t);
        setOutput<TORPEDO_AIMING_TRAJECTORY_TYPE>(TORPEDO_AIMING_TRAJECTORY, traj);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation
