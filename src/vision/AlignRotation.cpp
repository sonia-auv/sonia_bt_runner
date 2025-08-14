#include "sonia_bt_runner/vision/AlignRotation.hpp"

namespace vision{
    AlignRotation::AlignRotation(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::SyncActionNode(name, config), ros_node(node){
    
    }
    AlignRotation::~AlignRotation(){}
}