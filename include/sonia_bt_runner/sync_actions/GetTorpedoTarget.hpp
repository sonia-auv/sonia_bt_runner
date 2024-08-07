#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"

class GetTorpedoTarget : public BT::SyncActionNode
{
public:
	GetTorpedoTarget(const std::string &name, const BT::NodeConfig &config);

	static BT::PortsList providedPorts()
	{
		return {
			BT::InputPort<AiDetectionArray>("torpedo_objs"),
			BT::InputPort<int>("size", "0: smallest, 3: biggest"),
			BT::OutputPort<AiDetection>("selected_target")};
	}

	BT::NodeStatus tick() override;
private:
	float get_area(AiDetection obj);
};
