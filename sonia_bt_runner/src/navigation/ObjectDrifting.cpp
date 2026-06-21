#include "sonia_bt_runner/navigation/ObjectDrifting.hpp"

namespace navigation
{

    ObjectDrifting::ObjectDrifting(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), _ros_node(node)
    {

    }

    BT::NodeStatus ObjectDrifting::tick()
    {
        AiDetection det;

        // We get the detected object by the AI
        getInput("Detection", det);

        if(!getInput<Trajectory>("Trajectory"))
        {
            RCLCPP_INFO(_ros_node->get_logger(), "The Trajectory is not initialize");

            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(_ros_node->get_logger(), "Computing ObjectDrifting trajectory");
        
        // We compute a rotation to make the sub rotate at 90 degrees to the target
        TrajectoryPose t1;
        t1.positionX = 0.0;
        t1.positionY =0.0;
        t1.positionZ = 0.0;
        t1.orientationX = 0.0;
        t1.orientationY = 0.0;
        t1.orientationZ = det.angle_teta;
        t1.frame = 1;
        t1.speed = 0;
        t1.precision = 0;
        t1.long_rotation = false;

        RCLCPP_INFO(_ros_node->get_logger(), "position en x: %f", t1.positionX);
        RCLCPP_INFO(_ros_node->get_logger(), "position en y: %f", t1.positionY);
        RCLCPP_INFO(_ros_node->get_logger(), "position en z: %f", t1.positionZ);
        RCLCPP_INFO(_ros_node->get_logger(), "orientation en x: %f", t1.orientationX);
        RCLCPP_INFO(_ros_node->get_logger(), "orientation en y: %f", t1.orientationY);
        RCLCPP_INFO(_ros_node->get_logger(), "orientation en z: %f", t1.orientationZ);

        // We compute a translation in Y to go in front of the target
        TrajectoryPose t2;
        t2.positionX = 0.0;
	// https://en.wikipedia.org/wiki/Radian
#define DEG_TO_RAD(angle) ((angle) * M_PI / 180)
	t2.positionY = std::sin(DEG_TO_RAD(det.angle_teta)) * det.distance * -1.0;
#undef DEG_TO_RAD
        t2.positionZ = 0.0;
        t2.orientationX = 0.0;
        t2.orientationY = 0.0;
        t2.orientationZ = 0.0;
        t2.frame = 1;
        t2.speed = 0;
        t2.precision = 0;
        t2.long_rotation = false;

        RCLCPP_INFO(_ros_node->get_logger(), "position en x: %f", t2.positionX);
        RCLCPP_INFO(_ros_node->get_logger(), "position en y: %f", t2.positionY);
        RCLCPP_INFO(_ros_node->get_logger(), "position en z: %f", t2.positionZ);
        RCLCPP_INFO(_ros_node->get_logger(), "orientation en x: %f", t2.orientationX);
        RCLCPP_INFO(_ros_node->get_logger(), "orientation en y: %f", t2.orientationY);
        RCLCPP_INFO(_ros_node->get_logger(), "orientation en z: %f", t2.orientationZ);

        // We append the two new trajectory points to the sending trajectory
        Trajectory traj = getInput<Trajectory>("Trajectory").value();
        traj.trajectory.push_back(t1);
        traj.trajectory.push_back(t2);
        setOutput<Trajectory>("Trajectory", traj);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation
