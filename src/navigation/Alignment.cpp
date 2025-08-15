#include "sonia_bt_runner/navigation/Alignment.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>


// ----------------- Helper implementations -----------------

float bboxCenterX(const AiDetection& d)
{
    // Average all four corners (robust to minor skew)
    return 0.25f * (d.top_left_x + d.top_right_x + d.bottom_left_x + d.bottom_right_x);
}

AlignResult computeAlignmentHD720(const AiDetection& det, bool coords_are_normalized, bool alignement_by_translation)
{
    AlignResult out;

    // 1) detection center x in pixels
    float u_px = bboxCenterX(det);
    std::cout <<"center x pixel : "<< u_px << std::endl;

    // float u_px = coords_are_normalized ? (u / CameraInfoZedMiniHD720::width) : u;

    // 2) normalized lateral in [-1, 1] (no meters; control hint/UI)
    // out.norm_x = 2.0f * (u_px - CameraInfoZedMiniHD720::cx) / std::max(1, CameraInfoZedMiniHD720::width);
    // out.norm_x = 2.0f * (u_px - CameraInfoZedMiniHD720::cx) / std::max(1, CameraInfoZedMiniHD720::width);
    std::cout <<"center x pixel norm : "<< out.norm_x << std::endl;

    float fx=(CameraInfoZedMiniHD720::width/2)/(tan(CameraInfoZedMiniHD720::fovx*M_PI/180/2));
    ////////////////////////////////////WRONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNG : 3) bearing (radians) using fx/cx (positive = target to the right)

    // out.bearing_rad = std::atan((u_px - CameraInfoZedMiniHD720::cx) / CameraInfoZedMiniHD720::fx);
    // out.bearing_rad = out.norm_x*(CameraInfoZedMiniHD720::width/2) /(fx);
    out.bearing_rad = atan((u_px - CameraInfoZedMiniHD720::cx) /(fx));
    std::cout <<"bearing rad : "<< out.bearing_rad << std::endl;

    // out.bearing_rad = std::atan((u - CameraInfoZedMiniHD720::cx) / CameraInfoZedMiniHD720::fx);

    // 4) metric lateral if distance is valid: x = Z * (u - cx) / fx
    if (std::isfinite(det.distance) && det.distance > 0.0f)
    {
        if (alignement_by_translation){
            out.lateral_m=det.distance*sin(out.bearing_rad);
            std::cout <<"distance in meters for x : "<< out.lateral_m << std::endl;
        }

        // out.lateral_m = det.distance * (u_px - CameraInfoZedMiniHD720::cx) / CameraInfoZedMiniHD720::fx;
        out.has_metric = std::isfinite(out.lateral_m);
        std::cout <<"has_metric : "<< out.has_metric << std::endl;

    }
    
    return out;
}

// ----------------- Alignment node -----------------

namespace navigation
{

    // Alignment::Alignment(const std::string& name, const BT::NodeConfig& cfg) : BT::StatefulActionNode(name, cfg) {}
    Alignment::Alignment(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    :BT::StatefulActionNode(name, config), ros_node(node)
    {
        
    }

    // BT::PortsList Alignment::providedPorts()
    // {
    //     return {
    //         // Inputs
    //         InputPort<AiDetectionArray>("detections"),
    //         InputPort<bool>("normalized_coords"),
    //         InputPort<float>("alpha"),
    //         InputPort<bool>("alignement_by_translation"),

    //         // Outputs
    //         OutputPort<float>("lateral_m"),
    //         OutputPort<float>("bearing_rad"),
    //         OutputPort<float>("norm_x"),
    //         OutputPort<int>("has_metric"),
    //         OutputPort<float>("positionX"),
    //         OutputPort<float>("positionY"),
    //         OutputPort<float>("positionZ"),
    //         OutputPort<float>("orientationX"),
    //         OutputPort<float>("orientationY"),
    //         OutputPort<float>("orientationZ"),
    //         OutputPort<int>("frame"),
    //         OutputPort<int>("speed"),
    //         OutputPort<int>("precision"),
    //         OutputPort<bool>("longRotation"),
    //         // OutputPort<TrajectoryPose>("tp", output_state);


    //     };
    // }

    void Alignment::onHalted() {

    }

    BT::NodeStatus Alignment::onStart(){
        // arr = getInput<AiDetectionArray>("detections");
        // normalized= getInput<bool>("normalized_coords");
        // alpha = getInput<float>("alpha");
        // mode = getInput<bool>("alignement_by_translation");

        // if (arr->detection_array.empty())///////////////////////////////////////////////////////////////////////////////////ZARB   
        // {
        //     // publish safe defaults
        //     setOutput("bearing_rad", 0.0f);
        //     setOutput("norm_x", 0.0f);
        //     setOutput("has_metric", 0);
        //     std::cout <<"In Alignement Detection array empty"<< std::endl;

        //     return BT::NodeStatus::FAILURE;
        // }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus Alignment::onRunning()
    {
        AiDetectionArray arr;
        getInput("detections", arr);

        // if (!getInput("detections", arr) || arr.detection_array.empty())///////////////////////////////////////////////////////////////////////////////////ZARB   
        if (arr.detection_array.empty())///////////////////////////////////////////////////////////////////////////////////ZARB   
        {
            // publish safe defaults
            setOutput("bearing_rad", 0.0f);
            setOutput("norm_x", 0.0f);
            setOutput("has_metric", 0);
            std::cout <<"In Alignement Detection array empty"<< std::endl;

            return BT::NodeStatus::FAILURE;
        }

        bool normalized = false;//////////////////////////////////////////////////////////////////////////////////////////////////// PAS NECESSAIRE AU FINAL
        getInput("normalized_coords", normalized);//////////////////////////////////////////////////////////////////////////////////////////////////// PAS NECESSAIRE AU FINAL
        float alpha = -1.0f;
        getInput("alpha", alpha);
        std::cout <<"alpha : "<< alpha << std::endl;

        const bool do_smooth = (alpha > 0.0f && alpha <= 1.0f);

        // Assumption: array is pre-filtered for the object of interest → use first detection
        const AiDetection& det = arr.detection_array.front();
        bool mode = true;
        getInput("alignement_by_translation", mode);
        AlignResult res = computeAlignmentHD720(det, normalized,mode);

        int i;
        bool i_initialized=false;
        if (i_initialized==false)
            i=0;

        // Optional temporal smoothing (IIR) for stability
        if (do_smooth)
        {
            if (std::isfinite(prev_bearing_)){
                // std::cout <<"bearing rad before smoothing: "<< i << "step"<< res.bearing_rad << std::endl;
                res.bearing_rad = alpha * res.bearing_rad + (1.0f - alpha) * prev_bearing_;
                // std::cout <<"bearing rad after smoothing: "<< i << "step"<< res.bearing_rad << std::endl;
                i++;
            }
            if (std::isfinite(prev_normx_)) res.norm_x = alpha * res.norm_x + (1.0f - alpha) * prev_normx_;
            if (res.has_metric && std::isfinite(prev_lateral_)){
                std::cout <<"lateral_m before smoothing: "<< i << "step"<< res.lateral_m << std::endl;
                res.lateral_m = alpha * res.lateral_m + (1.0f - alpha) * prev_lateral_;
                std::cout <<"lateral_m after smoothing: "<< i << "step"<< res.lateral_m << std::endl;
        }

        }
        std::cout <<"center x pixel norm : "<< res.bearing_rad << std::endl;


        // Publish outputs
        setOutput("bearing_rad", res.bearing_rad);
        setOutput("norm_x", res.norm_x);
        std::cout <<"center x pixel norm : "<< res.bearing_rad << std::endl;

        setOutput("has_metric", res.has_metric ? 1 : 0);
        if (mode){
            // if (res.lateral_m<=error_positionY && res.lateral_m>=-error_positionY){
            //     res.in_interval=true;
            // }
            if(prev_bearing_ == res.bearing_rad)
                res.in_interval=true;

        }
        else {
            // if (res.bearing_rad<=error_orientationY && res.bearing_rad>=-error_orientationY){
            //     res.in_interval=true;
            // }
            
            if(prev_lateral_ = res.lateral_m)
                res.in_interval=true;

        }
        if (res.has_metric)
        {
            setOutput("lateral_m", res.lateral_m);
            prev_lateral_ = res.lateral_m;
        }

        prev_bearing_ = res.bearing_rad;
        std::cout <<"center x pixel norm : "<< res.bearing_rad << std::endl;

        prev_normx_ = res.norm_x;

        std::cout <<"center x pixel norm : "<< res.bearing_rad << std::endl;
        if (res.has_metric){
            TrajectoryPose t;
            float positionY = 0.0;
            if(mode){
                positionY=res.lateral_m;
                t.positionY = res.lateral_m;
                std::cout <<"translation y x pixel norm : "<< res.lateral_m << std::endl;
                std::cout <<"mode translation : "<< mode << "  translation Y: " << positionY << std::endl;
                }
            // setOutput("positionY", positionY);
            // float positionX = 0.0;
            // setOutput("positionX", positionX);
            // float positionZ = 0.0;
            // setOutput("positionZ", positionZ);
            // float orientationX = 0.0;
            // setOutput("orientationX", orientationX);
            // float orientationY = 0.0;
            // setOutput("orientationY", orientationY);
            float orientationZ = 0.0;
            if (!mode){
                orientationZ=res.bearing_rad;
                t.orientationZ = res.bearing_rad / M_PI * 180;
                std::cout <<"orientation z : "<< res.bearing_rad << std::endl;
                std::cout <<"mode translation : "<< mode << "  orientationZ: " << orientationZ << std::endl;
            }

            setOutput("orientationZ", orientationZ);
            // int frame = 1;
            t.frame = 1;
            // setOutput("frame", frame);
            // int speed = 2;
            // setOutput("speed", speed);
            t.speed = 2;
            // int precision = 0;
            // setOutput("precision", precision);
            // bool long_rotation = false;
            // setOutput("longRotation", long_rotation);
            Trajectory traj = getInput<Trajectory>("traj").value();
            traj.trajectory.push_back(t);
            setOutput<Trajectory>("traj", traj);
        }

        float error_positionY=0.1;
        float error_orientationY=0.1;
        // if (mode){
        //     // if (res.lateral_m<=error_positionY && res.lateral_m>=-error_positionY){
        //     //     res.in_interval=true;
        //     // }
        //     if(prev_bearing_ == res.bearing_rad)
        //         res.in_interval=true;

        // }
        // else {
        //     // if (res.bearing_rad<=error_orientationY && res.bearing_rad>=-error_orientationY){
        //     //     res.in_interval=true;
        //     // }
            
        //     if(prev_lateral_ = res.lateral_m)
        //         res.in_interval=true;

        // }
        //  TrajectoryPose output_state;
        // if (res.has_metric){
        //     output_state.positionX = 0.0;
        //     if(mode)
        //         output_state.positionX=res.lateral_m;
        //     output_state.positionY = 0.0;
        //     output_state.positionZ = 0.0;
        //     output_state.orientationX = 0.0;
        //     output_state.orientationY = 0.0;
        //     output_state.orientationZ = 0.0;
        //     if (!mode)
        //         output_state.orientationZ=res.bearing_rad;
        //     output_state.frame = 1;
        //     output_state.speed = 0;
        //     output_state.precision = 0;
        //     output_state.long_rotation = false;
        // }
        // setOutput("tp", output_state);


        // SUCCESS if metric lateral is available; otherwise RUNNING so parent can fall back to bearing-only logic
        return res.in_interval ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }

}  // namespace navigation
