#include "sonia_bt_runner/init/EnableDisableDVL.hpp"

namespace init{

    EnableDisableDVL::EnableDisableDVL(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), ros_node(node)
    {
        dvl_pub= ros_node->create_publisher<std_msgs::msg::Bool>("/provider_dvl/enable_disable_dvl",10);
    }
    EnableDisableDVL::~EnableDisableDVL() {}
    BT::NodeStatus EnableDisableDVL::tick(){
        std_msgs::msg::Bool dvl_state;
        BT::Expected<bool> data = getInput<bool>("dvl_input");

        dvl_state.data=data.value();
        dvl_pub->publish(dvl_state);
        return BT::NodeStatus::SUCCESS;
    }
}  // namespace init