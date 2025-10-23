#include "sonia_bt_runner/navigation/TorpedoAimingAlignement.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>

namespace navigation
{

    TorpedoAimingAlignement::TorpedoAimingAlignement(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), ros_node(node)
    {

    }

    BT::NodeStatus TorpedoAimingAlignement::tick()
    {
        AiDetectionArray arr;
        std::string launching_side;

        // We get the detected object by the AI
        getInput("Detections", arr);
        getInput("LaunchingSide", launching_side);

        if (arr.detection_array.empty())
        {
            // publish safe defaults
            std::cout <<"In Alignement Detection array empty"<< std::endl;

            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(ros_node->get_logger(), "Computing TorpedoAimingAlignment trajectory");
        // Assumption: array is pre-filtered for the object of interest → use first detection
        const AiDetection& det = arr.detection_array.front(); // A verifier si on peux renvoyer le plus proche a la place et non la premiere detection
        
        TrajectoryPose t;
        t.positionX = 0.0; // We don't move on the x axis
        if (launching_side == "portside")
        {
            t.positionY = det.distance_teta - CAMERA_TO_TORPIDO_PEPPER_OFFSET_X; // We move on the x axis
        }
        else
        {
            t.positionY = det.distance_teta - CAMERA_TO_TORPIDO_SALT_OFFSET_X; // We move on the x axis
        }
        t.positionZ = det.distance_beta - CAMERA_TO_TORPEDO_OFFSET_Y; // We move on the z axis
        t.orientationX = 0.0;       // We don't rotate on the x axis
        t.orientationY = 0.0;       // We don't rotate on the y axis
        t.orientationZ = 0.0;       // We don't rotate on the z axis
        t.frame = 1;                // We use the relative positionning
        t.speed = 0;
        t.precision = 0;            // We don't care about the precision
        t.long_rotation = false;    // We don't care about the long rotation

        RCLCPP_INFO(ros_node->get_logger(), "position en x: %f", t.positionX);
        RCLCPP_INFO(ros_node->get_logger(), "position en y: %f", t.positionY);
        RCLCPP_INFO(ros_node->get_logger(), "position en z: %f", t.positionZ);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en x: %f", t.orientationX);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en y: %f", t.orientationY);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en z: %f", t.orientationZ);
        RCLCPP_INFO(ros_node->get_logger(), "frame: %d", t.frame);
        RCLCPP_INFO(ros_node->get_logger(), "speed: %d", t.speed);
        RCLCPP_INFO(ros_node->get_logger(), "precision: %d", t.precision);
        RCLCPP_INFO(ros_node->get_logger(), "long rotation: %d", t.long_rotation);

        // We append the new pose to the trajectory
        Trajectory traj = getInput<Trajectory>("Trajectory").value();
        traj.trajectory.push_back(t);
        setOutput<Trajectory>("Trajectory", traj);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation