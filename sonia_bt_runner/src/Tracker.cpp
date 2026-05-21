#include "sonia_bt_runner/Tracker.hpp"

Tracker::Tracker(BT::Tree& tree, const std::shared_ptr<GoalHandle> goal)
    : BT::StatusChangeLogger(tree.rootNode()), _gl(goal)
    {
        _feedback =  std::make_shared<MissionControl::Feedback>();
    }

    void Tracker::callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status) 
    {
        try{
            _feedback->status = BT::toStr(status);
            _feedback->node_name= node.name();
            _feedback->uid = node.UID();
            _gl->publish_feedback(_feedback);
        }catch(const rclcpp::exceptions::RCLError &e){ } //catching for no goal instance warning after process ends
                
        std::this_thread::sleep_for(std::chrono::milliseconds(_THREAD_SLEEP_TIME));   
    }

    void Tracker::flush(){}