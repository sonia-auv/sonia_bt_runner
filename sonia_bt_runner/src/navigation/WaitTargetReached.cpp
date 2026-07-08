#include "sonia_bt_runner/navigation/WaitTargetReached.hpp"

using std::placeholders::_1;
namespace navigation{

    WaitTargetReached::WaitTargetReached(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node) 
    : BT::StatefulActionNode(name, config), _ros_node(node)
    {
        _timeout_pub = _ros_node->create_publisher<sonia_common_ros2::msg::MissionTimer>("/sonia_behaviors/timeout",5);
    }
    
     BT::NodeStatus WaitTargetReached::onStart(){
        
        _traj_complete = false;
        _time_diff = 0;
        _target_reached = false;
        _trajectory_done = true;
        _is_alive = true;

        getInput<WAIT_TARGET_REACHED_TIMEOUT_TYPE>(WAIT_TARGET_REACHED_TIMEOUT, _param_timeout);
    
        _launch_time = std::chrono::system_clock::now();
        _controller_sub =_ros_node->create_subscription<sonia_common_ros2::msg::MpcInfo>("/proc_control/controller_info", 1, std::bind(&WaitTargetReached::get_controller_info_callback, this,_1));
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus WaitTargetReached::onRunning(){
       if (!_is_alive)
        {
            return BT::NodeStatus::FAILURE;
        }

        if (_traj_complete)
        {
            std::chrono::duration<double> diff = std::chrono::system_clock::now() - _launch_time;
            _time_diff = diff.count();
        }

        if (_time_diff > _param_timeout || _target_reached)
        {
            if (_target_reached)
            {
                _timeout_pub->publish(missionTimerFunc("wait_target_reached", _param_timeout, std::chrono::system_clock::to_time_t(_launch_time), 2));
                RCLCPP_INFO(_ros_node->get_logger(), "Target Reached");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                _timeout_pub->publish(missionTimerFunc("wait_target_reached", _param_timeout, std::chrono::system_clock::to_time_t(_launch_time), 3));
                RCLCPP_INFO(_ros_node->get_logger(), "Target couldn't be reached");
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }
    void WaitTargetReached::onHalted(){}

    void WaitTargetReached::get_controller_info_callback(const sonia_common_ros2::msg::MpcInfo &msg){
        _target_reached = msg.target_reached;
        _trajectory_done = msg.is_trajectory_done;
        _is_alive = msg.is_mpc_alive;

        if (_trajectory_done != _trajectory_done_prev)
        {
            if (_trajectory_done)
            {
                _launch_time = std::chrono::system_clock::now();
                _traj_complete = true;
                _timeout_pub->publish(missionTimerFunc("wait_target_reached", _param_timeout, std::chrono::system_clock::to_time_t(_launch_time), 1));
                RCLCPP_INFO(_ros_node->get_logger(), "Trajectory Completed");
            }
            else
            {
                RCLCPP_INFO(_ros_node->get_logger(), "Trajectory has been received");    
            }
        }
        _trajectory_done_prev = _trajectory_done;
    }

    sonia_common_ros2::msg::MissionTimer WaitTargetReached::missionTimerFunc(std::string mission, float timeout, time_t uniqueID, int status)
    {
        sonia_common_ros2::msg::MissionTimer buffer;
        buffer.mission = mission;
        buffer.timeout = timeout;
        buffer.unique_id = std::to_string(uniqueID);
        buffer.status = status;
        return buffer;
    }

}
