#include "sonia_bt_runner/navigation/TorpedoDriftingAlignement.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>

namespace navigation
{

    TorpedoDriftingAlignement::TorpedoDriftingAlignement(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), ros_node(node)
    {

    }

    BT::NodeStatus TorpedoDriftingAlignement::tick()
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
        const AiDetection& det = arr.detection_array.front(); 
        
        // We compute a rotation to make the sub rotate at 90 degrees to the target
        TrajectoryPose t1;
        t1.positionX = 0.0;
        t1.positionY =0.0;
        t1.positionZ = 0.0;
        t1.orientationX = 0.0;
        t1.orientationY = 0.0;
        t1.orientationZ = -det.angle_teta;
        t1.frame = 1;
        t1.speed = 0;
        t1.precision = 0;
        t1.long_rotation = false;

        RCLCPP_INFO(ros_node->get_logger(), "position en x: %f", t1.positionX);
        RCLCPP_INFO(ros_node->get_logger(), "position en y: %f", t1.positionY);
        RCLCPP_INFO(ros_node->get_logger(), "position en z: %f", t1.positionZ);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en x: %f", t1.orientationX);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en y: %f", t1.orientationY);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en z: %f", t1.orientationZ);
        RCLCPP_INFO(ros_node->get_logger(), "frame: %d", t1.frame);
        RCLCPP_INFO(ros_node->get_logger(), "speed: %d", t1.speed);
        RCLCPP_INFO(ros_node->get_logger(), "precision: %d", t1.precision);
        RCLCPP_INFO(ros_node->get_logger(), "long rotation: %d", t1.long_rotation);

        // We compute a translation in Y to go in front of the target
        TrajectoryPose t2;
        t2.positionX = 0.0;
        t2.positionY = det.distance_teta;
        t2.positionZ = 0.0;
        t2.orientationX = 0.0;
        t2.orientationY = 0.0;
        t2.orientationZ = 0.0;
        t2.frame = 1;
        t2.speed = 0;
        t2.precision = 0;
        t2.long_rotation = false;

        RCLCPP_INFO(ros_node->get_logger(), "position en x: %f", t2.positionX);
        RCLCPP_INFO(ros_node->get_logger(), "position en y: %f", t2.positionY);
        RCLCPP_INFO(ros_node->get_logger(), "position en z: %f", t2.positionZ);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en x: %f", t2.orientationX);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en y: %f", t2.orientationY);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en z: %f", t2.orientationZ);
        RCLCPP_INFO(ros_node->get_logger(), "frame: %d", t2.frame);
        RCLCPP_INFO(ros_node->get_logger(), "speed: %d", t2.speed);
        RCLCPP_INFO(ros_node->get_logger(), "precision: %d", t2.precision);
        RCLCPP_INFO(ros_node->get_logger(), "long rotation: %d", t2.long_rotation);

        // We append the two new trajectory points to the sending trajectory
        Trajectory traj = getInput<Trajectory>("Trajectory").value();
        traj.trajectory.push_back(t1);
        traj.trajectory.push_back(t2);
        setOutput<Trajectory>("Trajectory", traj);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation