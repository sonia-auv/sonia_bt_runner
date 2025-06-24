#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace utils{

    template <typename T>
    class ActionServer{
        public:
            using GoalHandle = rclcpp_action::ServerGoalHandle<T>;
            using Callback = std::function<void(const std::shared_ptr<GoalHandle> goal_handle)>;

            ActionServer(rclcpp::Node::SharedPtr node, const std::string & name, Callback callback)
            : node_(node), server_(nullptr)
            {
                server_ = rclcpp_action::create_server<T>(
                    node_,
                    name,
                    std::bind(&ActionServer::handleGoal, this, _1,_2),
                    std::bind(&ActionServer::handleCancel, this, _1),
                    std::bind(&ActionServer::handleAccept, this, _1));

                callback_=callback;

            };
            
            rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const typename T::Goal> goal)
            {
                (void)goal;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };

            rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal)
            {
                (void)goal;
                return rclcpp_action::CancelResponse::ACCEPT;
            };
   
            void handleAccept(const std::shared_ptr<GoalHandle> goal)
            {
                std::thread([this, goal](){
                    callback_(goal);
                }).detach();
                
            };
        private:
            rclcpp::Node::SharedPtr node_;
            typename rclcpp_action::Server<T>::SharedPtr server_;
            Callback callback_;
            
    };
    
}


