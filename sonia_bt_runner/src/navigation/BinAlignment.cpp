#include "sonia_bt_runner/navigation/BinAlignment.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>

namespace navigation
{

    BinAlignment::BinAlignment(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), ros_node(node)
    {
        
    }

    BT::NodeStatus BinAlignment::tick()
    {
        AiDetectionArray arr;

        // We get the detected object by the AI
        getInput("Detections", arr);

        if (arr.detection_array.empty())
        {
            // publish safe defaults
            std::cout <<"In Alignement Detection array empty"<< std::endl;

            return BT::NodeStatus::FAILURE;
        }

        // Assumption: array is pre-filtered for the object of interest → use first detection
        const AiDetection& det = arr.detection_array.front(); // A verifier si on peux renvoyer le plus proche a la place et non la premiere detection
        
        // We compute the trajectory to do
        TrajectoryPose t;

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

        // We append the new trajectory point to the sending trajectory
        Trajectory traj = getInput<Trajectory>("Trajectory").value();
        traj.trajectory.push_back(t);
        setOutput<Trajectory>("Trajectory", traj);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation