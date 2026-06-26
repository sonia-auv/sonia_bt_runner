#pragma once

#include "sonia_bt_runner/vision/AbstractAiFilter.hpp"

#define SEARCH_SLALOM_SIDE "Side"
#define SEARCH_SLALOM_SIDE_PARAMS SEARCH_SLALOM_SIDE, "The side, the sub should take for the slalom (Left or Right)"
#define SEARCH_SLALOM_SIDE_TYPE std::string

namespace vision {
class SearchSlalom : public AbstractAiFilter {
public:
	SearchSlalom(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
fault;
	static BT::PortsList providedPorts()
        {
		auto default_ports = AbstractAiFilter::providedPorts();
		BT::PortsList new_ports = {
		    // Inputs 
                    BT::InputPort<SEARCH_SLALOM_SIDE_TYPE>(SEARCH_SLALOM_SIDE_PARAMS)
	       };

	       default_ports.merge(new_ports);

	       return default_ports;
       }

       BT::NodeStatus onStart() override;
protected:
       void handle_success() override;
       BT::NodeStatus get_detection_status() override;
       void ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) override;
private:
       SEARCH_SLALOM_SIDE_TYPE _side;
};
}
