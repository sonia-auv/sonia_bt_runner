#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"

namespace vision {
class SearchWhiteSlalom : public BT::StatefulActionNode {
	public:
		SearchWhiteSlalom(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
		~SearchWhiteSlalom() override = default;

            static BT::PortsList providedPorts() {
		return {
			// Input ports 
			BT::InputPort<AiDetection>("RedSlalomDetection"),
			BT::InputPort<std::string>("Side", "Left", "Left, Right"),

			// Output port
			BT::OutputPort<AiDetection>("WhiteSlalomDetection")
		};
	    }

	    BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;
	private:
            std::shared_ptr<rclcpp::Node> _ros_node;
};
}
