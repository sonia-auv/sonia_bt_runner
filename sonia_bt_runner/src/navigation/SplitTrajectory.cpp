#include "sonia_bt_runner/navigation/SplitTrajectory.hpp"

namespace navigation{
    SplitTrajectory::SplitTrajectory(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), _ros_node(node)
    {
    }

    BT::NodeStatus SplitTrajectory::tick()
    {
        // We get the information of the input
        auto offsetExpected = getInput<SPLIT_TRAJECTORY_OFFSET_TYPE>(SPLIT_TRAJECTORY_OFFSET_PARAM);
        SPLIT_TRAJECTORY_OFFSET_TYPE offset = 0;
        auto trajectory = getInput<SPLIT_TRAJECTORY_RECEIVED_TRAJECTORY_TYPE>(SPLIT_TRAJECTORY_RECEIVED_TRAJECTORY_PARAM).value();

        if (offsetExpected) {
            offset = offsetExpected.value();
        }

        // We verify if the offset is valid for an other trajectory
        if (offset >= trajectory.trajectory.size()) {
	    RCLCPP_INFO(_ros_node->get_logger(), "Over the offset, the block failed");
            return BT::NodeStatus::FAILURE;
        }

	RCLCPP_INFO(_ros_node->get_logger(), "Pass the offset is valid");

        SPLIT_TRAJECTORY_SPLITTED_TRAJECTORY_TYPE output_trajectory{};
        output_trajectory.trajectory.push_back(trajectory.trajectory[offset]);

        setOutput(SPLIT_TRAJECTORY_OFFSET_PARAM, ++offset);
        setOutput(SPLIT_TRAJECTORY_SPLITTED_TRAJECTORY_PARAM, output_trajectory);

        return BT::NodeStatus::SUCCESS;
    }
}
