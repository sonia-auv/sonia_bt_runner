#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/msg/detection.hpp"
#include "sonia_common_ros2/msg/detection_array.hpp"
#include "sonia_bt_runner/vision/AiFilter.hpp"

namespace vision {
	class SlalomAiFilter : public AiFilter {
		public:
			SlalomAiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
			~SlalomAiFilter() override = default;
			static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<std::string>("Side", "", "The passing side for the slalom"),
                    BT::InputPort<float>("Confidence", 0.6, "Ai confidence"),
                    BT::InputPort<float>("Max_depth", 25.0, "Maximum allowed depth"),
                    BT::InputPort<int>("Min_detections_before_success", 2, "Minumum number of frames with at least one detection before sending results. Need to be heigher than 2"),

                    BT::InputPort<int>("Max_frame_before_failing", 0, "Maximum allowed frames before failing the object research"),
                    BT::InputPort<float>("Max_time_before_failing_sec", 0.0, "Maximum allowed time in ms before failing the object research"),

                    // Outputs
                    BT::OutputPort<AiDetection>("Red_slalom"),
                    BT::OutputPort<AiDetection>("White_slalom")
                };
            }

		protected:
			std::vector<sonia_common_ros2::msg::Detection> _detection_array_white_slalom;
            BT::Expected<std::string> _side;
			void ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) override;
			void stock_input_parameters() override;
			bool initial_condition_verification() override;
			void setting_output() override;
		
	};
}
