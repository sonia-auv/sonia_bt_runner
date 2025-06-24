#include "sonia_bt_runner/MissionServer.hpp"

MissionServer::MissionServer()
    : Node("Mission_server")
    {
        
    } 
    
    MissionServer::~MissionServer()
    {}
    void MissionServer::init(){
        check_mission_server = std::make_shared<utils::ActionServer<sonia_common_ros2::action::WaitForTrue>>(
            shared_from_this(),
            "wait_for_true",
            [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<sonia_common_ros2::action::WaitForTrue>> goal){
                this->execute(goal);
            });
        RCLCPP_INFO(this->get_logger(), "Action Server up running");
    }
    void MissionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sonia_common_ros2::action::WaitForTrue>> goal)
    {

    }