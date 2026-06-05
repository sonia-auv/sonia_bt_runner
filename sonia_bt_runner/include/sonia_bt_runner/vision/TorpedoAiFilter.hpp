#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/vision/AiFilter.hpp"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_common_ros2/msg/detection.hpp"
#include "sonia_common_ros2/msg/detection_array.hpp"


namespace vision {
    class TorpedoAiFilter : public AiFilter {
        public:
            TorpedoAiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~TorpedoAiFilter() override = default;

        protected:
            void ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) override;
    };
}