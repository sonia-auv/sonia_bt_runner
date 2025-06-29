#include "sonia_bt_runner/MissionServer.hpp"
#include <functional>
using namespace std::placeholders;


MissionServer::MissionServer()
    : Node("Mission_server")
    {
        server_ = rclcpp_action::create_server<MissionControl>(
                    this,
                    "MissionControl",
                    std::bind(&MissionServer::handleGoal, this, _1,_2),
                    std::bind(&MissionServer::handleCancel, this, _1),
                    std::bind(&MissionServer::handleAccept, this, _1));

        RCLCPP_INFO(this->get_logger(), "Action Server up running");
    } 
    
    MissionServer::~MissionServer()
    {}
    void MissionServer::init(){
        
    }
    void MissionServer::execute(const std::shared_ptr<GoalHandle> goal)
    {

    }
    rclcpp_action::GoalResponse MissionServer::handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const MissionControl::Goal> goal){
        RCLCPP_INFO(this->get_logger(), "Received goal request with mission root %s", goal->mission);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse MissionServer::handleCancel(const std::shared_ptr<GoalHandle> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void MissionServer::handleAccept(const std::shared_ptr<GoalHandle> goal_handle){
        std::thread{std::bind(&MissionServer::execute, this, _1), goal_handle}.detach();
    }