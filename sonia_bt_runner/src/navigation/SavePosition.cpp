#include "sonia_bt_runner/navigation/SavePosition.hpp"
#include <chrono>
#include <thread>
#include <cmath>
#include <tuple>

using std::placeholders::_1;

namespace navigation{
    SavePosition::SavePosition(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)
    {
    }

    BT::NodeStatus SavePosition::tick()
    {
        position_sub =ros_node->create_subscription<geometry_msgs::msg::Pose>("/proc_control/current_target", 1, std::bind(&SavePosition::get_position_callback, this,_1));

        while (!_saved)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        Trajectory t;
        t.trajectory.push_back(_tp);
        setOutput("trajectory", t);

        return BT::NodeStatus::SUCCESS;
    }

    void SavePosition::get_position_callback(const geometry_msgs::msg::Pose &msg){
        

        _tp.positionX = msg.position.x;
        _tp.positionY = msg.position.y;
        _tp.positionZ = msg.position.z;

        std::tuple<double, double, double> rotation = SavePosition::quaternionToEuler(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
        _tp.orientationX = std::get<0>(rotation);
        _tp.orientationY = std::get<1>(rotation);
        _tp.orientationZ = std::get<2>(rotation);

        _tp.frame = 0;
        _tp.speed = 2;
        _tp.precision = 0;
        _tp.long_rotation = false;

        _saved = true;
    }

    std::tuple<double, double, double> SavePosition::quaternionToEuler(double w, double x, double y, double z)
    {
        // roll (x-axis rotation)
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        double roll = std::atan2(sinr_cosp, cosr_cosp);
    
        // pitch (y-axis rotation)
        double sinp = 2.0 * (w * y - z * x);
        double pitch;
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2.0, sinp); // clamp to 90° if out of range
        else
            pitch = std::asin(sinp);
    
        // yaw (z-axis rotation)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
    
        return std::make_tuple(roll, pitch, yaw);
    }
}