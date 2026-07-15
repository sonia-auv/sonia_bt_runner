#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/SlalomPointArray.hpp"

#define INITIALIZE_SLALOM_POINTS_POINTS "Points"
#define INITIALIZE_SLALOM_POINTS_POINTS_TYPE SlalomPointArray

namespace vision{
    class InitializeSlalomPoints : public BT::SyncActionNode
    {
    public:
        InitializeSlalomPoints(const std::string &name, const BT::NodeConfig &config);
        ~InitializeSlalomPoints() override = default;
        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<INITIALIZE_SLALOM_POINTS_POINTS_TYPE>(INITIALIZE_SLALOM_POINTS_POINTS),
            };
        }

        BT::NodeStatus tick() override;
    };
}
