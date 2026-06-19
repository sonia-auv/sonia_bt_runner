#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/msg/detection.hpp"
#include "sonia_common_ros2/msg/detection_array.hpp"
#include "sonia_bt_runner/vision/AiFilter.hpp"


#define SLALOM_AI_FILTER_SIDE_NAME "Side"
#define SLALOM_AI_FILTER_SIDE_TYPE std::string
#define SLALOM_AI_FILTER_RED_SLALOM_NAME "Red_slalom"
#define SLALOM_AI_FILTER_RED_SLALOM_NAME AiDetection
#define SLALOM_AI_FILTER_WHITE_SLALOM_NAME "White_slalom"
#define SLALOM_AI_FILTER_WHITE_SLALOM_NAME AiDetection
namespace vision {
	class SlalomAiFilter : public AiFilter {
		public:
			SlalomAiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
			~SlalomAiFilter() override = default;
			static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<AI_FILTER_CAMERA_TYPE>(AI_FILTER_CAMERA_NAME, 1, "1: Front, 0: Bottom"),
                    BT::InputPort<SLALOM_AI_FILTER_SIDE_TYPE>(SLALOM_AI_FILTER_SIDE_NAME, "", "The passing side for the slalom"),
                    BT::InputPort<AI_FILTER_CONFIDENCE_TYPE>(AI_FILTER_CONFIDENCE_NAME, 0.6, "Ai confidence"),
                    BT::InputPort<AI_FILTER_MAX_DEPTH_TYPE>(AI_FILTER_MAX_DEPTH_NAME, 25.0, "Maximum allowed depth"),
                    BT::InputPort<AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_TYPE>(AI_FILTER_MIN_DETECTIONS_BEFORE_SUCCESS_NAME, 2, "Minumum number of frames with at least one detection before sending results. Need to be heigher than 2"),

                    BT::InputPort<AI_FILTER_MAX_FRAME_BEFORE_FAILING_TYPE>(AI_FILTER_MAX_FRAME_BEFORE_FAILING_NAME, 0, "Maximum allowed frames before failing the object research"),
                    BT::InputPort<AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_TYPE>(AI_FILTER_MAX_TIME_BEFORE_FAILING_SEC_NAME, 0.0, "Maximum allowed time in ms before failing the object research"),

                    // Outputs
                    BT::OutputPort<SLALOM_AI_FILTER_RED_SLALOM_NAME>(SLALOM_AI_FILTER_RED_SLALOM_NAME),
                    BT::OutputPort<SLALOM_AI_FILTER_WHITE_SLALOM_NAME>(SLALOM_AI_FILTER_WHITE_SLALOM_NAME)
                };
            }

            BT::NodeStatus onStart() override;

		protected:
			std::vector<sonia_common_ros2::msg::Detection> _detection_array_white_slalom;
            BT::Expected<std::string> _side;
            
			void ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) override;
			void stock_input_parameters() override;
			void setting_output() override;
		
	};
}
