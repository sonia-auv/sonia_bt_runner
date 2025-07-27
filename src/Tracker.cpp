#include "sonia_bt_runner/Tracker.hpp"

Tracker::Tracker(BT::Tree& tree)
: BT::StatusChangeLogger(tree.rootNode()) 
{}

    void Tracker::callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status) 
    {
        if(status == BT::NodeStatus::SUCCESS)
        {
            std::cout << "✅ " << node.name() << " succeeded\n";
        }
        else if(status == BT::NodeStatus::FAILURE)
        {
            std::cout << "❌ " << node.name() << " failed\n";
        }
    }
    void Tracker::flush(){
        
    }