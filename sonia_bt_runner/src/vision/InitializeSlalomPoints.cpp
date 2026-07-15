#include "sonia_bt_runner/vision/InitializeSlalomPoints.hpp"

namespace vision{
    InitializeSlalomPoints::InitializeSlalomPoints(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)
    {
    }

    BT::NodeStatus InitializeSlalomPoints::tick()
    {
        setOutput(INITIALIZE_SLALOM_POINTS_POINTS, SlalomPointArray{});
        return BT::NodeStatus::SUCCESS;
    }
}
