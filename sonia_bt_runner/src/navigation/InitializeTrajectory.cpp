#include "sonia_bt_runner/navigation/InitializeTrajectory.hpp"

namespace navigation{
    InitializeTrajectory::InitializeTrajectory(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)
    {
    }

    BT::NodeStatus InitializeTrajectory::tick()
    {
        setOutput("trajectory", "");
        return BT::NodeStatus::SUCCESS;
    }
}