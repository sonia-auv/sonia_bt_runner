#include "sonia_bt_runner/navigation/FaceTheObject.hpp"

namespace navigation
{

    FaceTheObject::FaceTheObject(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
       : BT::SyncActionNode(name, config), _ros_node(node)
    {

    }
    BT::NodeStatus FaceTheObject::tick()
    {
        auto det = getInput<FACE_THE_OBJECT_DETECTION_TYPE>(FACE_THE_OBJECT_DETECTION_NAME).value();

        if(!getInput<FACE_THE_OBJECT_TRAJECTORY_TYPE>(FACE_THE_OBJECT_TRAJECTORY_NAME))
        {
            RCLCPP_INFO(_ros_node->get_logger(), "The Trajectory is not initialize");

            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(_ros_node->get_logger(), "Computing FaceTheObject trajectory");

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

        RCLCPP_INFO(_ros_node->get_logger(), "position en x: %f", t.positionX);
        RCLCPP_INFO(_ros_node->get_logger(), "position en y: %f", t.positionY);
        RCLCPP_INFO(_ros_node->get_logger(), "position en z: %f", t.positionZ);
        RCLCPP_INFO(_ros_node->get_logger(), "orientation en x: %f", t.orientationX);
        RCLCPP_INFO(_ros_node->get_logger(), "orientation en y: %f", t.orientationY);
        RCLCPP_INFO(_ros_node->get_logger(), "orientation en z: %f", t.orientationZ);

        // We append the new pose to the trajectory
        auto traj = getInput<FACE_THE_OBJECT_TRAJECTORY_TYPE>(FACE_THE_OBJECT_TRAJECTORY_NAME).value();
        traj.trajectory.push_back(t);
        setOutput<FACE_THE_OBJECT_TRAJECTORY_TYPE>(FACE_THE_OBJECT_TRAJECTORY_NAME, traj);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation