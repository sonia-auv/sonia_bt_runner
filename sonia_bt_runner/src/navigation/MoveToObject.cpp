#include "sonia_bt_runner/navigation/MoveToObject.hpp"

namespace navigation
{

    MoveToObject::MoveToObject(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
       : BT::SyncActionNode(name, config), ros_node(node)
    {

    }
    BT::NodeStatus MoveToObject::tick()
    {
        AiDetectionArray arr;
        float offset;

        // We get the detected object by the AI
        getInput("Detections", arr);
        getInput("Offset", offset);

        if (arr.detection_array.empty())
        {
            // publish safe defaults
            RCLCPP_INFO(ros_node->get_logger(), "Detection array empty");

            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(ros_node->get_logger(), "Computing MoveToObject trajectory");
        // Assumption: array is pre-filtered for the object of interest → use first detection
        const AiDetection& det = arr.detection_array.front(); // A verifier si on peux renvoyer le plus proche a la place et non la premiere detection

        TrajectoryPose t1;
        // We set the trajectory of the submarine to rotate on itself to face the object
        t1.positionX = 0.0; // We don't move on the x axis
        t1.positionY = 0.0; // We don't move on the x and y axis
        t1.positionZ = 0.0; // We don't move on the z axis
        t1.orientationX = 0.0; // We don't turn around the x axis
        t1.orientationY = 0.0; // We don't turn around the y axis
        t1.orientationZ = -det.angle_alpha;  // We turn around the z axis
        t1.frame = 1;                        // 0: relative to current position, 1
        t1.speed = 0;                        // Speed of the rotation
        t1.precision = 0;                    // Precision of the rotation
        t1.long_rotation = true;             // true: take the long way around, false:

        RCLCPP_INFO(ros_node->get_logger(), "position en x: %f", t1.positionX);
        RCLCPP_INFO(ros_node->get_logger(), "position en y: %f", t1.positionY);
        RCLCPP_INFO(ros_node->get_logger(), "position en z: %f", t1.positionZ);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en x: %f", t1.orientationX);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en y: %f", t1.orientationY);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en z: %f", t1.orientationZ);

        // We move forward to go to the object
        TrajectoryPose t2;
        t2.positionX = det.distance + offset; // We go to the object
        t2.positionY = 0.0; // We don't move lateraly
        t2.positionZ = 0.0; // We don't move up or down
        t2.orientationX = 0.0; // We don't rotate on the X axes
        t2.orientationY = 0.0; // We don't rotate on the Y axes
        t2.orientationZ = 0.0; // We rotate on the Z axes to face
        t2.frame = 1.0; // We use the frame in meter
        t2.speed = 0; // We use the default speed of the motion controller
        t2.precision = 0; // We use the default precision of the motion controller
        t2.long_rotation = false; // We don't use long rotation

        RCLCPP_INFO(ros_node->get_logger(), "position en x: %f", t2.positionX);
        RCLCPP_INFO(ros_node->get_logger(), "position en y: %f", t2.positionY);
        RCLCPP_INFO(ros_node->get_logger(), "position en z: %f", t2.positionZ);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en x: %f", t2.orientationX);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en y: %f", t2.orientationY);
        RCLCPP_INFO(ros_node->get_logger(), "orientation en z: %f", t2.orientationZ);

        // We append the new pose to the trajectory
        Trajectory traj = getInput<Trajectory>("Trajectory").value();
        traj.trajectory.push_back(t1);
        traj.trajectory.push_back(t2);
        setOutput<Trajectory>("Trajectory", traj);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation