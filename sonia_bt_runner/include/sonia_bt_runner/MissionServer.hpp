#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <chrono>
#include <filesystem>
#include <iostream>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/bt_observer.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "tinyxml2.h"

#include <std_msgs/msg/string.hpp>
#include "sonia_bt_runner/SoniaNodes.hpp"
#include "sonia_bt_runner/Tracker.hpp"
#include "sonia_common_ros2/action/mission_control.hpp"
#include "sonia_common_ros2/srv/mission_list_service.hpp"
#include "sonia_common_ros2/msg/node_status.hpp"

using namespace BT;
using MissionControl = sonia_common_ros2::action::MissionControl;
using GoalHandle = rclcpp_action::ServerGoalHandle<MissionControl>;

/**
 * @class MissionServer
 * @brief Handles mission requests for autonomous operations.
 */
class MissionServer : public rclcpp::Node{
    public:
        MissionServer();
        ~MissionServer() override = default;

        /**
         * @brief initializes the server by registering all existing BT nodes.
         */
        void init();

    private:
        /**
         * @brief loads all existing missions in a list.
         */
        void generateMissionList();

        /**
         * @brief Service to return a list of missions the server can run.
         * @param request Time of the event.
         * @param response node that changed status.
         */
        void grabMissionList(const std::shared_ptr<sonia_common_ros2::srv::MissionListService::Request> request, std::shared_ptr<sonia_common_ros2::srv::MissionListService::Response> response);
        
        /**
         * @brief Clears the factory of all registered BT nodes.
         * @param request Message to log according to when the function is called.
         */
        void clearFactory(const std::string log);

        /**
         * @brief Executes the requested mission once its accepted by the handler.
         * @param goal mission to be executed.
         */
        void execute(const std::shared_ptr<GoalHandle> goal);

        /**
         * @brief Publishes node information of its state and quality.
         */
        void publishStatus();

        /**
         * @brief Handles mission requested by the client.
         * @param uuid ID of the request.
         * @param goal mission request information.
         * @return The server's response to the request (accepted/rejected).
         */
        rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const MissionControl::Goal> goal);

        /**
         * @brief Handles cancel request.
         * @param goalhandle goal to be handled.
         * @return The server's response to the request (accepted/rejected).
         */
        rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goalhandle);

        /**
         * @brief Handles and executes the goal when accepted.
         * @param goal_handle goal to be executed.
         */
        void handleAccept(const std::shared_ptr<GoalHandle> goal_handle);

        std::string _name;
        BehaviorTreeFactory _factory;
        rclcpp_action::Server<MissionControl>::SharedPtr _server;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_status;
        rclcpp::Publisher<sonia_common_ros2::msg::NodeStatus>::SharedPtr _pub_node_status;
        rclcpp::Service<sonia_common_ros2::srv::MissionListService>::SharedPtr _fetch_missions_srv;
        rclcpp::TimerBase::SharedPtr _timer_node_status;

        Tree _tree;
        std::string _search_directory;
        NodeStatus _result;
        std::vector<std::string> _mission_list;
        sonia_common_ros2::msg::NodeStatus _node_status;

        const uint64_t _TICK_SLEEP_TIME = 66;
        
};