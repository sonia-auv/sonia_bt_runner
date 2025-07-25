#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <filesystem>
#include <iostream>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/bt_observer.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "sonia_bt_runner/SoniaNodes.hpp"
#include "sonia_common_ros2/action/mission_control.hpp"

using MissionControl = sonia_common_ros2::action::MissionControl;
using GoalHandle = rclcpp_action::ServerGoalHandle<MissionControl>;
//using Callback = std::function<void(const std::shared_ptr<GoalHandle> goal_handle)>;

class MissionServer : public rclcpp::Node{
    public:
        MissionServer();
        ~MissionServer();

        void init();
        void execute(const std::shared_ptr<GoalHandle> goal);
    private:
    
        rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const MissionControl::Goal> goal);
        rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goalhandle);
        void handleAccept(const std::shared_ptr<GoalHandle> goal);

        std::string name_;
        NodeStatus result_;
        BT::BehaviorTreeFactory factory_;
        rclcpp_action::Server<MissionControl>::SharedPtr server_;
        BT::Tree tree_;
        //BT::TreeObserver obs;
};