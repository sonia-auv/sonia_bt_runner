#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"

namespace vision{
    class AiFilter: public BT::StatefulActionNode{
        public:
            AiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~AiFilter();
            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<int>("Object", "Searched object"),
                    BT::InputPort<float>("Confidence", 0.8, "Ai confidence"),
                    BT::InputPort<int>("Min_count", "Minimum count"),
                    BT::InputPort<int>("Max_count", "Maximum count"),
                    BT::InputPort<int>("Max_frame", "maximum allowed frames"),
                    BT::InputPort<int>("Max_depth", "Maximum allowed depth")};
                    BT::OutputPort<AiDetectionArray>("ai_info");
            }

            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;
        private:
            std::shared_ptr<rclcpp::Node> ros_node;
    };

}