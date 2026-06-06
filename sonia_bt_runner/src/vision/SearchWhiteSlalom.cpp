#include "sonia_bt_runner/vision/SearchWhiteSlalom.hpp"

namespace vision {
SearchWhiteSlalom::SearchWhiteSlalom(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node) : BT::StatefulActionNode(name, config), _ros_node(node)
{}
}
