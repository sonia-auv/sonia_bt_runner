#include "sonia_bt_runner/vision/CheckDistance.hpp"

using std::placeholders::_1;
namespace vision{

    CheckDistance::CheckDistance(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), ros_node(node)
    {
        
    }
    CheckDistance::~CheckDistance(){}
    BT::NodeStatus CheckDistance::onStart(){
        object_concerned = getInput<AiDetectionArray>("Object");
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus CheckDistance::onRunning(){
        float max_depth;
        object_concerned2=object_concerned->detection_array.front();
        if (object_concerned2.classification=="gate")
            max_depth=0.6;
        else if (object_concerned2.classification=="gate-shark")
            max_depth=0.6;
        else if (object_concerned2.classification=="gate-sawfish")
            max_depth=0.6;
        else if (object_concerned2.classification=="torpedo-poster")
            max_depth=1;
        else if (object_concerned2.classification=="torpedo-target")
            max_depth=0.25;
        else if (object_concerned2.classification=="bin")
            max_depth=0.6;
        else if (object_concerned2.classification=="red-slalom")
            max_depth=0.6;
        else if (object_concerned2.classification=="table")
            max_depth=0.2;
        else 
            max_depth=1;

        if(object_concerned2.distance==0)
            translationX=0;
        else 
            translationX = object_concerned2.distance-max_depth;
        setOutput("TranslationX", translationX);
        return BT::NodeStatus::SUCCESS;

    }
    void CheckDistance::onHalted()
    {
    }
}  // namespace vision