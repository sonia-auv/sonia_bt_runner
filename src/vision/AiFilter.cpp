#include "sonia_bt_runner/vision/AiFilter.hpp"

namespace vision{
    AiFilter::AiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), ros_node(node)
    {

    }
    AiFilter::~AiFilter(){}
    BT::NodeStatus AiFilter::onStart(){

        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus AiFilter::onRunning(){
        
        return BT::NodeStatus::RUNNING;
    }
    void AiFilter::onHalted()
    {
    }
    
}