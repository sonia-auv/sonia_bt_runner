#include "sonia_bt_runner/navigation/FaceTheObject.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>

namespace navigation
{

    FaceTheObject::FaceTheObject(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
       : BT::SyncActionNode(name, config), ros_node(node)
    {

    }
    BT::NodeStatus FaceTheObject::tick()
    {
        AiDetectionArray arr;

        // We get the detected object by the AI
        getInput("Detections", arr);

        if (arr.detection_array.empty())
        {
            // publish safe defaults
            RCLCPP_INFO(ros_node->get_logger(), "Detection array empty");

            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(ros_node->get_logger(), "Computing FaceTheObject trajectory");
        // Assumption: array is pre-filtered for the object of interest → use first detection
        const AiDetection& det = arr.detection_array.front(); // A verifier si on peux renvoyer le plus proche a la place et non la premiere detection

        TrajectoryPose t;
        // We set the trajectory of the submarine to rotate on itself to face the object
        t.positionX = 0.0; // We don't move on the x axis
        t.positionY = 0.0; // We don't move on the x and y axis
        t.positionZ = 0.0; // We don't move on the z axis
        t.orientationX = 0.0; // We don't turn around the x axis
        t.orientationY = 0.0; // We don't turn around the y axis
        t.orientationZ = -det.angle_alpha;  // We turn around the z axis
        t.frame = 1;                        // 0: relative to current position, 1
        t.speed = 0;                        // Speed of the rotation
        t.precision = 0;                    // Precision of the rotation
        t.long_rotation = true;             // true: take the long way around, false:

        RCLCPP_INFO(ros_node->get_logger(), "position en x: %f", t.positionX);
        RCLCPP_INFO(ros_node->get_logger(), "position en y: %f", t.positionY);
        RCLCPP_INFO(ros_node->get_logger(), "position en z: %f", t.positionZ);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en x: %f", t.orientationX);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en y: %f", t.orientationY);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en z: %f", t.orientationZ);

        // We append the new pose to the trajectory
        Trajectory traj = getInput<Trajectory>("Trajectory").value();
        traj.trajectory.push_back(t);
        setOutput<Trajectory>("Trajectory", traj);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation