#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "ros/ros.h"

#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"

class CheckTranslationXYAlign : public BT::ConditionNode
{
public:
    CheckTranslationXYAlign(const std::string name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        const float error_margin_def = 0.05;
        return {
            BT::InputPort<AiDetection>("detection_object"),
            BT::InputPort<int>("target_x"),
            BT::InputPort<int>("target_y"),
            BT::InputPort<float>("distance"),
            BT::InputPort<float>("error_margin", error_margin_def, "percentage of error off the target (0-1)"),
            BT::BidirectionalPort<Trajectory>("traj")};
    }

    BT::NodeStatus tick() override;

private:
    const float _pixel_to_meters = 0.0024;
    const int _screen_height = 400; // pixels
    const int _screen_width = 600; // pixels
};
