/*#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"

/*namespace vision{
    class AiFilter: public BT::StatefulActionNode{
        public:
            AiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~AiFilter();
           /* static BT::PortsList providedPorts()
            {
                const std::string 
                const int activation_target_def = 0;
                const std::string 
                return {BT::InputPort<int>("activation_target", activation_target_def, "0: OFF, 1: FRONT, 2: BOTTOM, 3: BOTH")};
            }

            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;
        private:
            std::shared_ptr<rclcpp::Node> ros_node;
    };

}*/