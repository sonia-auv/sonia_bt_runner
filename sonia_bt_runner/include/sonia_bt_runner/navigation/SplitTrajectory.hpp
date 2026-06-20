#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/Trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

#define SPLIT_TRAJECTORY_OFFSET_PARAM "offset"
#define SPLIT_TRAJECTORY_RECEIVED_TRAJECTORY_PARAM "trajectory"
#define SPLIT_TRAJECTORY_SPLITTED_TRAJECTORY_PARAM "splittedTrajectory"
#define SPLIT_TRAJECTORY_OFFSET_TYPE size_t
#define SPLIT_TRAJECTORY_RECEIVED_TRAJECTORY_TYPE Trajectory
#define SPLIT_TRAJECTORY_SPLITTED_TRAJECTORY_TYPE Trajectory


namespace navigation{
    class SplitTrajectory : public BT::SyncActionNode
    {
        public:
            SplitTrajectory(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~SplitTrajectory() override = default;

            static BT::PortsList providedPorts()
            {

                return {
                    BT::BidirectionalPort<SPLIT_TRAJECTORY_OFFSET_TYPE>(SPLIT_TRAJECTORY_OFFSET_PARAM),
                    BT::InputPort<SPLIT_TRAJECTORY_RECEIVED_TRAJECTORY_TYPE>(SPLIT_TRAJECTORY_RECEIVED_TRAJECTORY_PARAM),
                    BT::OutputPort<SPLIT_TRAJECTORY_SPLITTED_TRAJECTORY_TYPE>(SPLIT_TRAJECTORY_SPLITTED_TRAJECTORY_PARAM)
                };
            }

            BT::NodeStatus tick() override;
       private:
            std::shared_ptr<rclcpp::Node> _ros_node;
    };
}
