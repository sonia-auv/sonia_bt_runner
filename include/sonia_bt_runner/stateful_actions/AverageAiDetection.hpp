#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common/DetectionArray.h"
#include "sonia_bt_runner/utils/AiDetection.hpp"
#include "ros/ros.h"

class AverageAiDetection : public BT::StatefulActionNode
{
public:
    AverageAiDetection(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("camera", 0, "0: Front, 1: Bottom"),
            BT::InputPort<int>("buffer_size", 5, "Number of detection to average"),
            BT::InputPort<int>("num_retries", 5, "Number of invalid detection allowed between good ones"),
            BT::InputPort<std::string>("classification"),
            BT::OutputPort<AiDetection>("average_obj")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}

private:
    void ai_info_front_callback(const sonia_common::DetectionArray &msg)
    {
        _last_front = msg;
        _last_front_fresh = true;
    }
    void ai_info_bottom_callback(const sonia_common::DetectionArray &msg)
    {
        _last_bottom = msg;
        _last_bottom_fresh = true;
    }

    sonia_common::Detection get_closest(const std::vector<sonia_common::Detection> obj_arr);

    AiDetection get_average();

    ros::NodeHandle _nh;
    ros::Subscriber _ai_info_front_sub;
    ros::Subscriber _ai_info_bottom_sub;
    bool _last_front_fresh;
    sonia_common::DetectionArray _last_front;
    bool _last_bottom_fresh;
    sonia_common::DetectionArray _last_bottom;

    int _buffer_size;
    int _num_retries;
    int _ai_not_found_counter;
    int _camera;
    std::string _classification;
    std::vector<sonia_common::Detection> _buffer;
};