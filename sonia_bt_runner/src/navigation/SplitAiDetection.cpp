#include "sonia_bt_runner/navigation/SplitAiDetection.hpp"

namespace navigation{
    SplitAiDetection::SplitAiDetection(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), _ros_node(node)
    {
    }

    BT::NodeStatus SplitAiDetection::tick()
    {
        // We get the information of the input
        auto offsetExpected = getInput<SPLIT_AI_DETECTION_OFFSET_TYPE>(SPLIT_AI_DETECTION_OFFSET_PARAM);
        SPLIT_AI_DETECTION_OFFSET_TYPE offset = 0;
        auto aiDetection = getInput<SPLIT_AI_DETECTION_RECEIVED_TYPE>(SPLIT_AI_DETECTION_RECEIVED_DETECTION_PARAM).value();

        if (offsetExpected) {
            offset = offsetExpected.value();
        }

        // We verify if the offset is valid for an other trajectory
        if (offset >= aiDetection.size()) {
	        RCLCPP_INFO(_ros_node->get_logger(), "Over the offset, the block failed");
            return BT::NodeStatus::FAILURE;
        }

	RCLCPP_INFO(_ros_node->get_logger(), "Pass the offset is valid");

        SPLIT_AI_DETECTION_SPLITTED_DETECTION_TYPE output_detection{};
        output_detection = aiDetection[offset];

        setOutput(SPLIT_AI_DETECTION_OFFSET_PARAM, ++offset);
        setOutput(SPLIT_AI_DETECTION_SPLITTED_DETECTION_PARAM, output_detection);

        return BT::NodeStatus::SUCCESS;
    }
}
