#include "sonia_bt_runner/Tracker.hpp"

Tracker::Tracker(BT::Tree& tree, const std::shared_ptr<GoalHandle> goal)
    : BT::StatusChangeLogger(tree.rootNode()), gl(goal)
    {
        feedback =  std::make_shared<MissionControl::Feedback>();
    }

    void Tracker::callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status) 
    {
        try{
            feedback->status = BT::toStr(status);
            feedback->node_name= node.name();
            feedback->uid = node.UID();
            gl->publish_feedback(feedback);
        }catch(const rclcpp::exceptions::RCLError &e){ } //catching for no goal instance warning after process ends
                
        std::this_thread::sleep_for(std::chrono::milliseconds(_THREAD_SLEEP_TIME));   
    }

    void Tracker::flush(){}