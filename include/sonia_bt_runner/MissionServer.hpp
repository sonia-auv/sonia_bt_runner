#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "sonia_bt_runner/utils/ActionServer.hpp"
#include "sonia_common_ros2/action/wait_for_true.hpp"

class MissionServer : public rclcpp::Node{
    public:
        MissionServer();
        ~MissionServer();

        void init();
        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sonia_common_ros2::action::WaitForTrue>> goal);
    private:
        std::shared_ptr<utils::ActionServer<sonia_common_ros2::action::WaitForTrue>> check_mission_server;
        
};