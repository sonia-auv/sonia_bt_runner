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
        object_concerned = getInput<sonia_common_ros2::msg::Detection>("Object");
        growing_width = getInput<bool>("Growing_width"); //for the first time we check the width it needs to be false
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus CheckBoundingBox::onRunning(){
        if(growing_width){
        //     if ( previous_width.value()>=current_width.value()){
        //         growing_width=false;
        //         return BT::NodeStatus::FAILURE;
        //     }
        //     else (previous_width.value()>current_width.value()){
        //         growing_width=true;
        //         setOutput("New_width", current_width);
        //         return BT::NodeStatus::SUCCESS;
        //     }
        // }
        // else{
        //     if ( previous_width.value()<=current_width.value()){
        //         growing_width=false;
        //         setOutput("New_width", current_width);
        //         return BT::NodeStatus::SUCCESS;
        //     }
        //     else (previous_width.value()>current_width.value()){
        //         growing_width=true;
        //         setOutput("New_width", current_width);
        //         return BT::NodeStatus::SUCCESS;
        //     }
        }

        // if(growing_width){
        //     if ( previous_width.value()>=current_width.value()){
        //         growing_width=false;
        //         return BT::NodeStatus::FAILURE;
        //     }
        //     else (previous_width.value()>current_width.value()){
        //         growing_width=true;
        //         setOutput("New_width", current_width);
        //         return BT::NodeStatus::SUCCESS;
        //     }
        // }
        // else{
        //     if ( previous_width.value()<=current_width.value()){
        //         growing_width=false;
        //         setOutput("New_width", current_width);
                // return BT::NodeStatus::SUCCESS;
        return BT::NodeStatus::SUCCESS;

        //     }
        //     else (previous_width.value()>current_width.value()){
        //         growing_width=true;
        //         setOutput("New_width", current_width);
                // return BT::NodeStatus::SUCCESS;
        //     }
        // }

    }
    void CheckBoundingBox::onHalted()
    {
    }
}  // namespace vision