#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"

class GetTorpedoTarget ": public BT::SyncActionNode
{
public:
	GetTorpedoTarget(const std::string name, const BT::NodeConfig &config);

	static BT::portsList providedPorts()
	{
		return {
			BT::InputPort<AiDetectionArray>("torpedo_objs"),
			BT::InputPort<int>("size", "0: smallest, 3: biggest")};
	}

	BT::NodeStatus tick() override;
	
};
