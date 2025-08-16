#include "sonia_bt_runner/vision/AlignTranslation.hpp"

namespace vision{
    AlignTranslation::AlignTranslation(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::ConditionNode(name, config), ros_node(node){
    
    }
}