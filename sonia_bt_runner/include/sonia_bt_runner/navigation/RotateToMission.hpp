#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "sonia_bt_runner/navigation/MainAlignment.hpp"

namespace navigation
{

    class RotateToMission : public BT::StatefulActionNode
    {
        public:
            RotateToMission(const std::string &name, const BT::NodeConfig &config,std::shared_ptr<rclcpp::Node> node);
            ~RotateToMission()=default;
            static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<AiDetectionArray>("detections"),

                    // Outputs
                    BT::BidirectionalPort<Trajectory>("trajectory"),
                };
            }

            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;

        private:
            std::shared_ptr<rclcpp::Node> ros_node;
            int valid;
            std::chrono::system_clock::time_point _time_launch;
            BT::Expected<AiDetectionArray> arr;
            BT::Expected<bool> mode;
            BT::Expected<bool>camera;
            // Previous outputs for optional temporal smoothing
            // float prev_lateral_ = std::numeric_limits<float>::quiet_NaN();   ???
            // float prev_bearing_ = std::numeric_limits<float>::quiet_NaN();   ???
            // float prev_normx_ = std::numeric_limits<float>::quiet_NaN();     ???


    };

}  // namespace navigation
