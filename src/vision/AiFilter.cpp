#include "sonia_bt_runner/vision/AiFilter.hpp"

using std::placeholders::_1;
namespace vision{
    AiFilter::AiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), ros_node(node)
    {
        ai_front_filter_sub = ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/front/classif",1, std::bind(&AiFilter::ai_front_filter_callback, this, _1));
        ai_bottom_filter_sub = ros_node->create_subscription<sonia_common_ros2::msg::DetectionArray>("/proc_vision/bottom/classif",1, std::bind(&AiFilter::ai_bottom_filter_callback, this, _1));
    }
    AiFilter::~AiFilter(){}
    BT::NodeStatus AiFilter::onStart(){

        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus AiFilter::onRunning(){
        
        return BT::NodeStatus::RUNNING;
    }
    void AiFilter::onHalted()
    {
    }


    void AiFilter::ai_front_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) {

    }
    void AiFilter::ai_bottom_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) {

    }
}  // namespace vision