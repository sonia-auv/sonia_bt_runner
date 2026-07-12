#pragma once

#define AI_FILTER_DETECTED_OBJECT "Detected_object"
#define AI_FILTER_DETECTED_OBJECT_TYPE AiDetection

namespace vision {
class HasDetection : public BT::SyncActionNode {
     HasDetection(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    ~HasDetection() override = default;
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<AI_FILTER_DETECTED_OBJECT>(AI_FILTER_DETECTED_OBJECT_TYPE),
	};
    }

    BT::NodeStatus tick() override;
	
private:
    std::shared_ptr<rclcpp::Node> _ros_node;
};
}
