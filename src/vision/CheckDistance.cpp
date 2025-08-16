#include "sonia_bt_runner/vision/CheckDistance.hpp"

using std::placeholders::_1;
namespace vision{

    CheckDistance::CheckDistance(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), ros_node(node)
    {
        
    }
    CheckDistance::~CheckDistance(){}
    BT::NodeStatus CheckDistance::onStart(){
        object_concerned = getInput<sonia_common_ros2::msg::Detection>("Object");
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus CheckDistance::onRunning(){
        float max_depth;
        if (object_concerned->class_name=="gate")
            max_depth=0.6;
        else if (object_concerned->class_name=="gate-shark")
            max_depth=0.6;
        else if (object_concerned->class_name=="gate-sawfish")
            max_depth=0.6;
        else if (object_concerned->class_name=="torpedo-poster")
            max_depth=1;
        else if (object_concerned->class_name=="torpedo-target")
            max_depth=0.25;
        else if (object_concerned->class_name=="bin")
            max_depth=0.6;
        else if (object_concerned->class_name=="red-slalom")
            max_depth=0.6;
        else if (object_concerned->class_name=="table")
            max_depth=0.2;
        else 
            max_depth=1;

        translationX = object_concerned->distance-max_depth;
        setOutput("TranslationX", translationX);
        return BT::NodeStatus::SUCCESS;

    }
    void CheckDistance::onHalted()
    {
    }
}  // namespace vision