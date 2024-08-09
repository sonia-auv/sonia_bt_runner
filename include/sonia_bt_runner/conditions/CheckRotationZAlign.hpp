#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "ros/ros.h"

#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"

class CheckRotationZAlign : public BT::ConditionNode
{
public:
    CheckRotationZAlign(const std::string name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        const float error_margin_def = 0.05;
        const float _pixel_to_degrees_def = 0.0666;

        return {
            BT::InputPort<AiDetection>("detection_object"),
            BT::InputPort<int>("target_x"),
            BT::InputPort<float>("error_margin", error_margin_def, "percentage of error off the target (0-1)"),
            BT::InputPort<float>("pixel_to_degrees", _pixel_to_degrees_def, "0.0666 for the auv8, 0.11 for the auv7"),
            BT::BidirectionalPort<Trajectory>("traj")};
    }

    BT::NodeStatus tick() override;

private:
    const int _screen_height = 400; // pixels
    const int _screen_width = 600; // pixels
};