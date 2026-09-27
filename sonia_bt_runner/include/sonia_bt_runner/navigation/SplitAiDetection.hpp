#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "sonia_bt_runner/utils/Point.hpp"

#define SPLIT_AI_DETECTION_OFFSET_PARAM "offset"
#define SPLIT_AI_DETECTION_RECEIVED_DETECTION_PARAM "detection"
#define SPLIT_AI_DETECTION_SPLITTED_DETECTION_PARAM "splittedDetections"
#define SPLIT_AI_DETECTION_OFFSET_TYPE size_t
#define SPLIT_AI_DETECTION_RECEIVED_TYPE std::vector<Point>
#define SPLIT_AI_DETECTION_SPLITTED_DETECTION_TYPE Point


namespace navigation{
    class SplitAiDetection : public BT::SyncActionNode
    {
        public:
            SplitAiDetection(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~SplitAiDetection() override = default;

            static BT::PortsList providedPorts()
            {

                return {
                    BT::BidirectionalPort<SPLIT_AI_DETECTION_OFFSET_TYPE>(SPLIT_AI_DETECTION_OFFSET_PARAM),
                    BT::InputPort<SPLIT_AI_DETECTION_RECEIVED_TYPE>(SPLIT_AI_DETECTION_RECEIVED_DETECTION_PARAM),
                    BT::OutputPort<SPLIT_AI_DETECTION_SPLITTED_DETECTION_TYPE>(SPLIT_AI_DETECTION_SPLITTED_DETECTION_PARAM)
                };
            }

            BT::NodeStatus tick() override;
       private:
            std::shared_ptr<rclcpp::Node> _ros_node;
    };
}
