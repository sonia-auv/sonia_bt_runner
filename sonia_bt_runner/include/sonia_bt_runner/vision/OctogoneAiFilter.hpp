#pragma once

#include "sonia_bt_runner/vision/AbstractAiFilter.hpp"
#include "sonia_bt_runner/utils/AiDetection.hpp"

#define OCTOGONE_AI_FILTER_DETECTED_OBJECT "Detected_object"
#define OCTOGONE_AI_FILTER_DETECTED_OBJECT_TYPE AiDetection

namespace vision{
    class OctogoneAiFilter: public AbstractAiFilter {
        public:
            OctogoneAiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~OctogoneAiFilter() override = default;
            static BT::PortsList providedPorts()
            {
	       auto default_ports = AbstractAiFilter::providedPorts();
	       BT::PortsList new_ports = {
		    // Outputs
                    BT::OutputPort<OCTOGONE_AI_FILTER_DETECTED_OBJECT_TYPE>(OCTOGONE_AI_FILTER_DETECTED_OBJECT)
	       };

	       default_ports.merge(new_ports);

	       return default_ports;
            }

            BT::NodeStatus onStart() override;
        protected: 
	        void handle_success() override;
	        BT::NodeStatus get_detection_status() override;
            void ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) override;
            AiDetection detection_average();         

	        OCTOGONE_AI_FILTER_DETECTED_OBJECT_TYPE _object_class;
            std::vector<sonia_common_ros2::msg::Detection> _detection_array;
    };
}
