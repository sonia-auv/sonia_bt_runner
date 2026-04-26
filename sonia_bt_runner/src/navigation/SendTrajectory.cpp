#include "sonia_bt_runner/navigation/SendTrajectory.hpp"

using std::placeholders::_1;
namespace navigation{
    SendTrajectory::SendTrajectory(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node) 
    : BT::StatefulActionNode(name, config), ros_node(node), valid(0), _time_launch(std::chrono::system_clock::now())
    {
        planner_pub = ros_node->create_publisher<sonia_common_ros2::msg::PoseArray>("/proc_planner/send_pose_array",10);
        depth_sub = ros_node->create_subscription<std_msgs::msg::Float32>("/provider_depth/depth", 10, std::bind(&SendTrajectory::update_depth, this, _1));

    }

    BT::NodeStatus SendTrajectory::onStart(){
        Trajectory t;
        getInput<Trajectory>("trajectory", t);
        sonia_common_ros2::msg::PoseArray array_to_send;
        int interpolation = 0;
        getInput<int>("interpolation", interpolation);
        
        float max_depth=1.5;

        array_to_send.interpolation_method = interpolation;
        for (size_t i = 0; i < t.trajectory.size(); i++)
        {
            sonia_common_ros2::msg::Pose ap;
            ap.position.x = t.trajectory[i].positionX;
            ap.position.y = t.trajectory[i].positionY;
            if (t.trajectory[i].frame==1 or t.trajectory[i].frame==2 or t.trajectory[i].frame==4){
                if (_depth_val+t.trajectory[i].positionZ<max_depth)
                    ap.position.z = t.trajectory[i].positionZ;
                else{
                    ap.position.z = max_depth;
                }
            }
            ap.orientation.x = t.trajectory[i].orientationX;
            ap.orientation.y = t.trajectory[i].orientationY;
            ap.orientation.z = t.trajectory[i].orientationZ;
            ap.frame = t.trajectory[i].frame;
            ap.speed = t.trajectory[i].speed;
            ap.fine = t.trajectory[i].precision;
            ap.rotation = t.trajectory[i].long_rotation;
            array_to_send.poses.push_back(ap);
        }
        planner_pub->publish(array_to_send);
        _time_launch = std::chrono::system_clock::now();
        planner_sub =ros_node->create_subscription<std_msgs::msg::Int8>("/proc_planner/is_waypoints_valid", 10, std::bind(&SendTrajectory::isWaypointValid, this,_1));
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus SendTrajectory::onRunning(){
        std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - _time_launch;
        // if (elapsed_time.count() > 5) // Ajouter le valid dans la condition, cette partie est à modifier pour garder de la vitesse
        // {
        //     if (valid == 0)
        //     {
        //         return BT::NodeStatus::SUCCESS;
        //     }
        //     else
        //         return BT::NodeStatus::FAILURE;
        // }
        if (valid == 0)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else if (elapsed_time.count() > 5)
        {
            return BT::NodeStatus::FAILURE;
        }
    
        return BT::NodeStatus::RUNNING;
    }
    void SendTrajectory::onHalted(){}

    void SendTrajectory::isWaypointValid(const std_msgs::msg::Int8 &msg){
        valid= msg.data;
    }
    void SendTrajectory::update_depth(const std_msgs::msg::Float32::ConstSharedPtr &msg)
    {
        _depth_val = msg->data;
    }
}
