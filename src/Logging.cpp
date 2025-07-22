#include "sonia_bt_runner/Logging.hpp"

Logging::Logging(BT::Tree& tree): BT::StatusChangeLogger(tree){

}

void Logging::callback(BT::Duration timestamp,const BT::TreeNode& node, BT::NodeStatus prev, BT::NodeStatus status){
    std::cout <<"Node: " <<node.name()
                  <<"from: " <<toStr(prev)
                  <<"to: "   <<toStr(status)<<std::endl;
}