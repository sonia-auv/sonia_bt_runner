#pragma once

#include "sonia_bt_runner/vision/AbstractAiFilter.hpp"
#include "sonia_bt_runner/utils/AiDetection.hpp"

#define SEARCH_CLOSEST_SLALOM_OBJECT_CLASS "Object_class"
#define SEARCH_CLOSEST_SLALOM_CLASS_PARAMS SEARCH_CLOSEST_SLALOM_OBJECT_CLASS, "Searched object"
#define SEARCH_CLOSEST_SLALOM_OBJECT_CLASS_TYPE std::string

#define SEARCH_CLOSEST_SLALOM_CLOSEST_DETECTED_OBJECT "Closest_object"
#define SEARCH_CLOSEST_SLALOM_CLOSEST_DETECTED_OBJECT_PARAMS SEARCH_CLOSEST_SLALOM_CLOSEST_DETECTED_OBJECT, "The closest detected object"
#define SEARCH_CLOSEST_SLALOM_CLOSEST_DETECTED_OBJECT_TYPE AiDetection

#define SEARCH_CLOSEST_SLALOM_CLOSEST_OBJECT_ANGLE "Closest_object_angle"
#define SEARCH_CLOSEST_SLALOM_CLOSEST_OBJECT_ANGLE_PARAMS SEARCH_CLOSEST_SLALOM_CLOSEST_OBJECT_ANGLE, "The closest detection angle"
#define SEARCH_CLOSEST_SLALOM_CLOSEST_OBJECT_ANGLE_TYPE float

#define SEARCH_CLOSEST_SLALOM_ACTUAL_ANGLE "Actual_angle"
#define SEARCH_CLOSEST_SLALOM_ACTUAL_ANGLE_PARAMS SEARCH_CLOSEST_SLALOM_ACTUAL_ANGLE, "The actual angle of the sub"
#define SEARCH_CLOSEST_SLALOM_ACTUAL_ANGLE_TYPE float

namespace vision{
    class SearchClosestSlalom: public AbstractAiFilter {
        public:
            SearchClosestSlalom(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~SearchClosestSlalom() override = default;
            static BT::PortsList providedPorts()
            {
                auto default_ports = AbstractAiFilter::providedPorts();
                BT::PortsList new_ports = {
                    // Inputs
                    BT::InputPort<SEARCH_CLOSEST_SLALOM_OBJECT_CLASS_TYPE>(SEARCH_CLOSEST_SLALOM_CLASS_PARAMS),
                    BT::InputPort<SEARCH_CLOSEST_SLALOM_ACTUAL_ANGLE_TYPE>(SEARCH_CLOSEST_SLALOM_ACTUAL_ANGLE_PARAMS),
                    // Bidirectionnals
                    BT::BidirectionalPort<SEARCH_CLOSEST_SLALOM_CLOSEST_DETECTED_OBJECT_TYPE>(SEARCH_CLOSEST_SLALOM_CLOSEST_DETECTED_OBJECT_PARAMS),
                    BT::BidirectionalPort<SEARCH_CLOSEST_SLALOM_CLOSEST_OBJECT_ANGLE_TYPE>(SEARCH_CLOSEST_SLALOM_CLOSEST_OBJECT_ANGLE_PARAMS)
                };
                default_ports.merge(new_ports);

	            return default_ports;
            }

            BT::NodeStatus onStart() override;

        protected:
            SEARCH_CLOSEST_SLALOM_OBJECT_CLASS_TYPE _object_class;
	    std::optional<SEARCH_CLOSEST_SLALOM_CLOSEST_DETECTED_OBJECT_TYPE> _closest_object_detected;
            SEARCH_CLOSEST_SLALOM_CLOSEST_OBJECT_ANGLE_TYPE _closest_object_angle;
            SEARCH_CLOSEST_SLALOM_ACTUAL_ANGLE_TYPE _actual_angle;
            AiDetection detection_average();
            virtual void handle_success();
            std::vector<sonia_common_ros2::msg::Detection> _detection_array;
            BT::NodeStatus get_detection_status() override;

            void ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) override;
        
    };
}
