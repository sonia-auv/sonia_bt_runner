#include "sonia_bt_runner/vision/HasDetection.hpp"

namespace vision {
BT::NodeStatus HasDetection::tick()
{
	HasDetection::HasDetection(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node) : BT::SyncActionNode(name, config), _ros_node(node) {}
	auto detected_object = getInput<AI_FILTER_DETECTED_OBJECT_TYPE>(AI_FILTER_DETECTED_OBJECT).value();

	if (detected_object.classification == "") {
		return BT::NodeStatus::FAILURE;
	}

	return BT::NodeStatus::SUCCESS;
}
}
