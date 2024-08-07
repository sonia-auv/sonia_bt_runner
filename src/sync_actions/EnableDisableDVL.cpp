#include "sonia_bt_runner/sync_actions/EnableDisableDVL.hpp"

EnableDisableDVL::EnableDisableDVL(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
{
    _dvl_pub = _nh.advertise<std_msgs::Bool>("/provider_dvl/enable_disable_ping", 100);
}

BT::NodeStatus EnableDisableDVL::tick()
{
    std_msgs::Bool dvl_state;

    bool data = false;
    getInput<bool>("dvl_input", data);
    if (data)
    {
        ROS_INFO("DVL is started");
    }
    else if (!data)
    {
        ROS_INFO("DVL is stopped");
    }
    else
    {
        ROS_INFO("Invalid input. input is boolean");
        return BT::NodeStatus::FAILURE;
    }
    dvl_state.data = data;
    _dvl_pub.publish(dvl_state);
    return BT::NodeStatus::SUCCESS;
}