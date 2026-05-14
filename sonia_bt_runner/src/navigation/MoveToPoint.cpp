#include "sonia_bt_runner/navigation/MoveToPoint.hpp"

namespace navigation {

    MoveToPoint::MoveToPoint(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), ros_node(node)
    {
        
    }

    BT::NodeStatus MoveToPoint::tick()
    {
        Point point_retour;
        Trajectory traj;

        getInput("point", point_retour);
        getInput("trajectory", traj);

        TrajectoryPose point_retour_pose;
        point_retour_pose.orientationX = 0;
        point_retour_pose.orientationY = 0;
        point_retour_pose.orientationZ = 0;
        point_retour_pose.positionX = point_retour.x;
        point_retour_pose.positionY = point_retour.y;
        point_retour_pose.positionZ = point_retour.z;
        point_retour_pose.frame = 3;
        point_retour_pose.speed = 0;
        point_retour_pose.precision = 0;
        point_retour_pose.long_rotation = false;

        traj.trajectory.push_back(point_retour_pose);
        setOutput("trajectory", traj);
        return BT::NodeStatus::SUCCESS;
    }
}