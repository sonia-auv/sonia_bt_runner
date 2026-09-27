#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"

#include <rclcpp/rclcpp.hpp>
#include <string>

#define AI_FILTER_DETECTED_OBJECT "Detected_object"
#define AI_FILTER_DETECTED_OBJECT_TYPE AiDetection

namespace vision {
class HasDetection : public BT::SyncActionNode {
public:
     HasDetection(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    ~HasDetection() override = default;
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<AI_FILTER_DETECTED_OBJECT_TYPE>(AI_FILTER_DETECTED_OBJECT)
	};
    }

    BT::NodeStatus tick() override;
	
private:
    std::shared_ptr<rclcpp::Node> _ros_node;
};
}
