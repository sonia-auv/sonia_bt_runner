#include "sonia_bt_runner/navigation/SavePoint.hpp"

using std::placeholders::_1;

namespace navigation {

    SavePoint::SavePoint(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
        : BT::StatefulActionNode(name, config), ros_node(node), pose_msg(), msg_received{false}
    {
        rclcpp::QoS qos(10);
        pose_sub = ros_node->create_subscription<sonia_common_ros2::msg::Pose>("/proc_control/current_target", qos, std::bind(&SavePoint::pose_call_back,this, _1));
    }

    BT::NodeStatus SavePoint::onStart()
    {
        _launch_time = std::chrono::system_clock::now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus SavePoint::onRunning()
    {
        std::chrono::duration<double> diff = std::chrono::system_clock::now() - _launch_time;
        time_diff = diff.count();
        if (msg_received) {
            Point point_retour;
            point_retour.x = pose_msg.position.x;
            point_retour.y = pose_msg.position.y;
            point_retour.z = pose_msg.position.z;
            setOutput("point", point_retour);
            return BT::NodeStatus::SUCCESS;
        }
        else if (time_diff > 5.0)
        {
            RCLCPP_WARN(ros_node->get_logger(), "The current target wasn't published. Verify the control node");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void SavePoint::onHalted()
    {

    }

    void SavePoint::pose_call_back(const sonia_common_ros2::msg::Pose &msg)
    {
        pose_msg = msg;
        msg_received = true;
    }
    
}
