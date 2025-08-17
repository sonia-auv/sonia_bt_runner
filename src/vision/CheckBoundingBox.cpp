#include "sonia_bt_runner/vision/CheckBoundingBox.hpp"

using std::placeholders::_1;
namespace vision{

    CheckBoundingBox::CheckBoundingBox(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), ros_node(node)
    {
        
    }
    CheckBoundingBox::~CheckBoundingBox(){}
    BT::NodeStatus CheckBoundingBox::onStart(){
        previous_width = getInput<int>("Previous_width");
        object_concerned = getInput<AiDetectionArray>("Object");
        growing_width = getInput<bool>("Growing_width"); //for the first time we check the width it needs to be false
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus CheckBoundingBox::onRunning(){
        object_concerned2=object_concerned->detection_array.front();
        current_width=(object_concerned2.top_left_x+object_concerned2.top_right_x+object_concerned2.bottom_left_x+object_concerned2.bottom_right_x)/2;
        if(growing_width){
            if ( previous_width.value()>=current_width.value()){
                growing_width=false;
                setOutput("New_width", current_width);
                return BT::NodeStatus::SUCCESS;
            }
            else {
                growing_width=true;
                setOutput("New_width", current_width);
                return BT::NodeStatus::FAILURE;
            }
        }
        else{
            if ( previous_width.value()<=current_width.value()){
                growing_width=false;
            }
            else{
                growing_width=true;
            }
            setOutput("New_width", current_width);
            setOutput("New_width_growth", growing_width);

            return BT::NodeStatus::FAILURE;

        }

    }
    void CheckBoundingBox::onHalted()
    {
    }
}  // namespace vision