#include "sonia_bt_runner/sensors/DepthCheck.hpp"

using std::placeholders::_1;

namespace sensors
{
    DepthCheck::DepthCheck(const std::string &name, std::shared_ptr<rclcpp::Node> node) : BT::ConditionNode(name, {}), ros_node(node), _depth_val{0.1}
    {
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        depth_sub = ros_node->create_subscription<std_msgs::msg::Float32>("/provider_depth/depth", qos, std::bind(&DepthCheck::update_status, this, _1));
    }

    BT::NodeStatus DepthCheck::tick()
    {
        BT::Expected<float> target = getInput<float>("Target");
        BT::Expected<bool> is_smaller = getInput<bool>("IsSmaller");

        if (is_smaller.value())
        {
            if (_depth_val <= target.value())
            {
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            if (_depth_val >= target.value())
            {
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }
    }

    void DepthCheck::update_status(const std_msgs::msg::Float32::ConstPtr &msg)
    {
        _depth_val = msg->data;
    }
} // namespace sensors
