#pragma once

#include "sonia_bt_runner/vision/AbstractAiFilter.hpp"
#include "sonia_bt_runner/utils/AiDetection.hpp"

#define AI_FILTER_OBJECT_CLASS "Object_class"
#define AI_FILTER_OBJECT_CLASS_PARAMS AI_FILTER_OBJECT_CLASS, "Searched object"
#define AI_FILTER_OBJECT_CLASS_TYPE std::string

#define AI_FILTER_DETECTED_OBJECT "Detected_object"
#define AI_FILTER_DETECTED_OBJECT_TYPE AiDetection

namespace vision{
    class AiFilter: public AbstractAiFilter {
        public:
            AiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~AiFilter() override = default;
            static BT::PortsList providedPorts()
            {
	       auto default_ports = AbstractAiFilter::providedPorts();
	       BT::PortsList new_ports = {
		    // Inputs
		    BT::InputPort<AI_FILTER_OBJECT_CLASS_TYPE>(AI_FILTER_OBJECT_CLASS_PARAMS),
		    // Outputs
                    BT::OutputPort<AI_FILTER_DETECTED_OBJECT_TYPE>(AI_FILTER_DETECTED_OBJECT)
	       };

	       default_ports.merge(new_ports);

	       return default_ports;
            }

            BT::NodeStatus onStart() override;
        protected: 
	    void handle_success() override;
	    BT::NodeStatus get_detection_status() override;
            void ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) override;

            void applicate_box_plot_to_detections();
            AiDetection detection_average();         

	    AI_FILTER_OBJECT_CLASS_TYPE _object_class;
            std::vector<sonia_common_ros2::msg::Detection> _detection_array;
    };
}
