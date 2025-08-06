#include "sonia_bt_runner/Tracker.hpp"

Tracker::Tracker(BT::Tree& tree, const std::shared_ptr<GoalHandle> goal)
    : BT::StatusChangeLogger(tree.rootNode()), gl(goal)
    {
        feedback =  std::make_shared<MissionControl::Feedback>();
    }

    Tracker::~Tracker(){}
    void Tracker::callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status) 
    {
        
        feedback->status = BT::toStr(status);
        feedback->node_name= node.name();
        feedback->uid = node.UID();

        //std::cout << "status: "<<BT::toStr(status)<<" name: "<<node.name()<<" UID: "<<node.UID()<<std::endl;
        gl->publish_feedback(feedback);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));   
    }

    void Tracker::flush(){

    }