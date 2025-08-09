#include "sonia_bt_runner/vision/AiFilter.hpp"

using std::placeholders::_1;
namespace vision{
    AiFilter::AiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), ros_node(node)
    {
            
    }
    AiFilter::~AiFilter(){}
    BT::NodeStatus AiFilter::onStart(){
        _object=getInput<std::string>("Object");
        BT::Expected<int> cam = getInput<int>("Camera");

        //chose camera
        if(cam.value())
            ai_filter_sub = ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/front/classif", 1, std::bind(&AiFilter::ai_filter_callback, this, _1));
        else    
            ai_filter_sub = ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/bottom/classif", 1, std::bind(&AiFilter::ai_filter_callback, this, _1));    
            return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus AiFilter::onRunning(){
        BT::Expected<int> buffer_size= getInput<int>("Camera");
        //if the array size remains small than the buffer size return
        if(_dection_array.size()< buffer_size.value())
            return BT::NodeStatus::RUNNING;

        AiDetection avg_object;

        float avg_bot_left_x, avg_bot_left_y, avg_bot_right_x, avg_bot_right_y= 0.0;
        float avg_top_left_x, avg_top_left_y, avg_top_right_x, avg_top_right_y= 0.0;

        for (auto det: _dection_array){
            avg_top_left_x += det.top_left_x;
            avg_top_left_y += det.top_left_y;
            avg_top_right_x += det.top_right_x;
            avg_top_right_x += det.top_right_x;

            avg_bot_left_x += det.bottom_left_x;
            avg_bot_left_y += det.bottom_left_y;
            avg_bot_right_x += det.bottom_right_x;
            avg_bot_right_x += det.bottom_right_x;
        } 
        avg_object.bottom_left_x=avg_bot_left_x/buffer_size.value();
        avg_object.bottom_left_y=avg_bot_left_y/buffer_size.value();
        avg_object.bottom_right_x=avg_bot_right_x/buffer_size.value();
        avg_object.bottom_right_y=avg_bot_right_y/buffer_size.value();

        avg_object.top_left_x=avg_top_left_x/buffer_size.value();
        avg_object.top_left_y=avg_top_left_y/buffer_size.value();
        avg_object.top_right_x=avg_top_right_x/buffer_size.value();
        avg_object.top_right_y=avg_top_right_y/buffer_size.value();

        avg_object.classification= _dection_array.at(0).class_name;
        setOutput("average_obj", avg_object);
        return BT::NodeStatus::SUCCESS;

    }
    void AiFilter::onHalted()
    {
    }

    void AiFilter::ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) {
        BT::Expected<float> confidence = getInput<int>("Confidence");
        for (auto msg_obj: msg.detected_object){
            if(msg_obj.class_name == _object.value()&&msg_obj.confidence>=confidence.value()){
                _dection_array.push_back(msg_obj);
            }
        }     
    }
}  // namespace vision