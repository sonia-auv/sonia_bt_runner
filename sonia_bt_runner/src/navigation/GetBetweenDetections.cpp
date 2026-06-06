#include "sonia_bt_runner/navigation/GetBetweenDetections.hpp"

namespace navigation
{
	GetBetweenDetections::GetBetweenDetections(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), _ros_node(node)
    {}

    BT::NodeStatus GetBetweenDetections::tick()
    {
	AiDetection detectionA, detectionB;
	
	if (!getInput<AiDetection>("DetectionA", detectionA) ||
	    !getInput<AiDetection>("DetectionB", detectionB)) {
	    RCLCPP_INFO(_ros_node->get_logger(), "One of the inputs is not present");

	   return BT::NodeStatus::FAILURE;
	}

	int positionY = (detectionA.top_left_y + detectionB.top_left_y) / 2;

	setOutput<float>("PositionY", positionY);

	return BT::NodeStatus::SUCCESS;
    }
}
