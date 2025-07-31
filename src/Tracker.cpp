#include "sonia_bt_runner/Tracker.hpp"

Tracker::Tracker(BT::Tree& tree, std::shared_ptr<MissionControl::Feedback> feedback)
    : BT::StatusChangeLogger(tree.rootNode()), fb(feedback)
    {

    }

    Tracker::~Tracker(){}
    void Tracker::callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status) 
    {
        std::string st= BT::toStr(status);
        std::string name = node.name();

        
        /*if(status == BT::NodeStatus::SUCCESS)
        {
            //fb->status = node.name() + " curr: "+ BT::toStr(status);
            std::cout << "✅ " << node.name() <<  " "<< BT::toStr(status)<<" \n";
        }
        else if(status == BT::NodeStatus::RUNNING)
        {
            //fb->status = node.name() + " curr: "+ BT::toStr(status);
            std::cout << "❎" << node.name() << " "<< BT::toStr(status) <<"\n";
        }
        else if(status == BT::NodeStatus::FAILURE)
        {
            //fb->status = node.name() + " curr: "+ BT::toStr(status);
            std::cout << "❌ " << node.name() << " "<< BT::toStr(status) <<"\n";
        }*/
        fb->status = name + " curr: "+ st;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
       //tracker_= node.name() + " prev: "+ BT::toStr(prev_status)+ " curr: "+ BT::toStr(status);
    }

    void Tracker::flush(){

    }