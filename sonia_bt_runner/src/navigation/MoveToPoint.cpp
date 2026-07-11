#include "sonia_bt_runner/navigation/MoveToPoint.hpp"

namespace navigation {

    MoveToPoint::MoveToPoint(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), _ros_node(node)
    {
        
    }

    BT::NodeStatus MoveToPoint::tick()
    {
        MOVE_TO_POINT_POINT_TYPE point_retour;
        MOVE_TO_POINT_TRAJECTORY_TYPE traj;
        MOVE_TO_POINT_OFFSET_POSITION_X_TYPE offset_position_x;
        MOVE_TO_POINT_OFFSET_POSITION_Y_TYPE offset_position_y;
        MOVE_TO_POINT_OFFSET_POSITION_Z_TYPE offset_position_z;

        
        if(!getInput(MOVE_TO_POINT_POINT, point_retour))
        {
            RCLCPP_INFO(_ros_node->get_logger(), "The point is not specified");

            return BT::NodeStatus::FAILURE;
        }

        if(!getInput<MOVE_TO_POINT_TRAJECTORY_TYPE>(MOVE_TO_POINT_TRAJECTORY))
        {
            RCLCPP_INFO(_ros_node->get_logger(), "The Trajectory is not initialize");

            return BT::NodeStatus::FAILURE;
        }
        getInput(MOVE_TO_POINT_TRAJECTORY, traj);
        getInput(MOVE_TO_POINT_OFFSET_POSITION_X, offset_position_x);
        getInput(MOVE_TO_POINT_OFFSET_POSITION_Y, offset_position_y);
        getInput(MOVE_TO_POINT_OFFSET_POSITION_Z, offset_position_z);

        TrajectoryPose point_retour_pose;
        point_retour_pose.orientationX = 0;
        point_retour_pose.orientationY = 0;
        point_retour_pose.orientationZ = 0;
        point_retour_pose.positionX = point_retour.x + offset_position_x;
        point_retour_pose.positionY = point_retour.y + offset_position_y;
        point_retour_pose.positionZ = point_retour.z + offset_position_z;
        point_retour_pose.frame = 3;
        point_retour_pose.speed = 0;
        point_retour_pose.precision = 0;
        point_retour_pose.long_rotation = false;

        traj.trajectory.push_back(point_retour_pose);
        setOutput(MOVE_TO_POINT_TRAJECTORY, traj);
        return BT::NodeStatus::SUCCESS;
    }
}
