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

float bboxCenterY(const AiDetection& d)
{
    // Average all four corners (robust to minor skew)
    return 0.25f * (d.top_left_y + d.top_right_y + d.bottom_left_y + d.bottom_right_y);
}


AlignResult computeAlignmentHD720(const AiDetection& det, bool alignement_by_translation)
{
    AlignResult out;

    // 1) detection center x in pixels
    float u_px = bboxCenterX(det);
    std::cout <<"center x pixel : "<< u_px << std::endl;

    // float u_px = coords_are_normalized ? (u / CameraInfoZedMiniHD720::width) : u;

    // 2) normalized lateral in [-1, 1] (no meters; control hint/UI)
    // out.norm_x = 2.0f * (u_px - CameraInfoZedMiniHD720::cx) / std::max(1, CameraInfoZedMiniHD720::width);
    // out.norm_x = 2.0f * (u_px - CameraInfoZedMiniHD720::cx) / std::max(1, CameraInfoZedMiniHD720::width);
    std::cout <<"center dist x pixel norm : "<< (u_px - CameraInfoZedMiniHD720::cx)  << std::endl;

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
        

    }
    out.has_metric = true;
    std::cout <<"has_metric : "<< out.has_metric << std::endl;
    return out;
}

AlignResult computeAlignmentY(const AiDetection& det, bool alignement_by_translation)
{
    AlignResult out;

    // 1) detection center x in pixels
    float u_py = bboxCenterY(det);
    std::cout <<"center x pixel : "<< u_py << std::endl;

    // float u_px = coords_are_normalized ? (u / CameraInfoZedMiniHD720::width) : u;

    // 2) normalized lateral in [-1, 1] (no meters; control hint/UI)
    // out.norm_x = 2.0f * (u_px - CameraInfoZedMiniHD720::cx) / std::max(1, CameraInfoZedMiniHD720::width);
    // out.norm_x = 2.0f * (u_px - CameraInfoZedMiniHD720::cx) / std::max(1, CameraInfoZedMiniHD720::width);
    std::cout <<"center dist x pixel norm : "<< (u_py - CameraInfoZedMiniHD720::c_y)  << std::endl;

    float fy=(CameraInfoZedMiniHD720::height/2)/(tan(CameraInfoZedMiniHD720::fov_y*M_PI/180/2));
    ////////////////////////////////////WRONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNG : 3) bearing (radians) using fx/cx (positive = target to the right)

    // out.bearing_rad = std::atan((u_px - CameraInfoZedMiniHD720::cx) / CameraInfoZedMiniHD720::fx);
    // out.bearing_rad = out.norm_x*(CameraInfoZedMiniHD720::width/2) /(fx);
    out.bearing_rad = atan((u_py - CameraInfoZedMiniHD720::c_y) /(fy));
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
        

    }
    out.has_metric = true;
    std::cout <<"has_metric : "<< out.has_metric << std::endl;
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
            std::cout <<"In Alignement Detection array empty"<< std::endl;

            return BT::NodeStatus::FAILURE;
        }

        // Assumption: array is pre-filtered for the object of interest → use first detection
        const AiDetection& det = arr.detection_array.front();
        bool mode = true;
        getInput("alignement_by_translation", mode);
        AlignResult res = computeAlignmentHD720(det,mode);
        AlignResult res_y = computeAlignmentY(det,mode);


        if (res.has_metric){
            TrajectoryPose t;
            t.positionY = 0.0;
            if(mode){
                t.positionY = res.lateral_m;
                std::cout <<"translation y x pixel norm : "<< res.lateral_m << std::endl;
                }
            // setOutput("positionY", positionY);
            t.positionX = 0.0;
            if (camera==false)
                t.positionX = res_y.lateral_m;
            // setOutput("positionX", positionX);
            t.positionZ = 0.0;
            // setOutput("positionZ", positionZ);
            t.orientationX = 0.0;
            // setOutput("orientationX", orientationX);
            t.orientationY = 0;
            // setOutput("orientationY", orientationY);
            t.orientationZ = 0;
            std::cout << "position en x: "<<t.positionX;
            if (!mode){
                t.orientationZ = res.bearing_rad / M_PI * 180;
                std::cout <<"orientation z : "<< res.bearing_rad << std::endl;

            }
            // int frame = 1;
            t.frame = 1;
            // setOutput("frame", frame);
            // int speed = 2;
            // setOutput("speed", speed);
            t.speed = 2;
            t.precision = 0;
            t.long_rotation=false;
            // setOutput("precision", precision);
            // bool long_rotation = false;
            // setOutput("longRotation", long_rotation);
            std::cout << "position en x: "<<t.positionX<< std::endl;
            std::cout<< "position en y: "<<t.positionY<< std::endl;
            std::cout << "position en z: "<<t.positionZ<< std::endl;
            std::cout << "Orientation en x: "<<t.orientationX<< std::endl;
            std::cout << "Orientation en y: "<<t.orientationY<< std::endl;
            std::cout << "Orientation en z: "<<t.orientationZ<< std::endl;
            std::cout << "frame: "<<t.frame<< std::endl;
            std::cout << "speed: "<<t.speed<< std::endl;
            std::cout << "precision: "<<t.precision<< std::endl;
            std::cout << "long rotation: "<<t.long_rotation<< std::endl;
            Trajectory traj = getInput<Trajectory>("traj").value();
            traj.trajectory.push_back(t);
            setOutput<Trajectory>("traj", traj);
        }


        // SUCCESS if metric lateral is available; otherwise RUNNING so parent can fall back to bearing-only logic
        return res.has_metric ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }

}  // namespace navigation
