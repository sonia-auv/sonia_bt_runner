#include "sonia_bt_runner/stateful_actions/AverageAiDetection.hpp"

AverageAiDetection::AverageAiDetection(const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config), _buffer_size(0), _num_retries(8), _ai_not_found_counter(0), _camera(0)
{
    _ai_info_front_sub = _nh.subscribe("/proc_vision/front/classification", 10, &AverageAiDetection::ai_info_front_callback, this);
    _ai_info_bottom_sub = _nh.subscribe("/proc_vision/bottom/classification", 10, &AverageAiDetection::ai_info_bottom_callback, this);
}

BT::NodeStatus AverageAiDetection::onStart()
{
    getInput<int>("buffer_size", _buffer_size);
    getInput<int>("num_retries", _num_retries);
    getInput<std::string>("classification", _classification);
    getInput<int>("camera", _camera);

    _buffer.clear();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AverageAiDetection::onRunning()
{
    ros::Duration(1).sleep();
    ros::spinOnce();

    sonia_common::DetectionArray value;

    if (_camera == 0) // FRONT
    {
        if (_last_front_fresh == false)
        {
            return BT::NodeStatus::RUNNING;
        }
        value = _last_front;
        _last_front_fresh = false;
    }
    else if (_camera == 1) // BOTTOM
    {
        if (_last_bottom_fresh == false)
        {
            return BT::NodeStatus::RUNNING;
        }
        value = _last_bottom;
        _last_bottom_fresh = false;
    }
    else
    {
        ROS_INFO("BAD_CAM");
        return BT::NodeStatus::FAILURE;
    }

    // Extract all good classifications
    std::vector<sonia_common::Detection> tmp_arr;
    for (size_t i = 0; i < value.detected_object.size(); i++)
    {
        if (value.detected_object[i].class_name == _classification)
        {
            tmp_arr.push_back(value.detected_object[i]);
        }
    }

    // if arr is empty, continue
    if (tmp_arr.size() < 1)
    {
        _ai_not_found_counter += 1;
        if (_ai_not_found_counter > _num_retries)
        {
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    // at least one object found
    _ai_not_found_counter = 0;

    sonia_common::Detection selected_obj;
    if (tmp_arr.size() > 1)
    {
        // Many objects found, get closest
        selected_obj = get_closest(tmp_arr);
    }
    else
    {
        selected_obj = tmp_arr[0];
    }

    _buffer.push_back(selected_obj);

    if (_buffer.size() >= _buffer_size)
    {
        AiDetection avg_res = get_average();
        setOutput("average_obj", avg_res);
        BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

sonia_common::Detection AverageAiDetection::get_closest(const std::vector<sonia_common::Detection> obj_arr)
{
    sonia_common::Detection selected_obj;
    selected_obj = obj_arr[0];

    for (size_t i = 1; i < obj_arr.size(); i++)
    {
        if (obj_arr[i].distance < selected_obj.distance)
        {
            selected_obj = obj_arr[i];
        }
    }
    return selected_obj;
}

AiDetection AverageAiDetection::get_average()
{
    AiDetection avg_obj;

    float avg_dist = 0;
    float avg_top = 0;
    float avg_left = 0;
    float avg_bot = 0;
    float avg_right = 0;

    for (size_t i = 0; i < _buffer.size(); i++)
    {
        auto tmp = _buffer[i];
        avg_dist += tmp.distance;
        avg_top += tmp.top;
        avg_left += tmp.left;
        avg_bot += tmp.bottom;
        avg_right += tmp.right;
    }
    avg_obj.distance = avg_dist / _buffer_size;
    avg_obj.top = avg_top / _buffer_size;
    avg_obj.left = avg_left / _buffer_size;
    avg_obj.bottom = avg_bot / _buffer_size;
    avg_obj.right = avg_right / _buffer_size;
    avg_obj.classification = _buffer[0].class_name;
    return avg_obj;
}