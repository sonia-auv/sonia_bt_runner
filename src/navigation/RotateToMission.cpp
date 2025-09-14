#include "sonia_bt_runner/navigation/RotateToMission.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>

namespace navigation
{

    RotateToMission::RotateToMission(const std::string &name, const BT::NodeConfig &config,std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), ros_node(node), valid(0), _time_launch(std::chrono::system_clock::now())
    {

    }
    static BT::PortsList providedPorts()
    {
        return {
            // Inputs
            BT::InputPort<AiDetectionArray>("detections"),

            // Outputs
            BT::BidirectionalPort<Trajectory>("traj"),
        };
    }

    // BT::PortsList Alignment::providedPorts()frontres
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
    //         OutputPort<int>("speed"),AlignResult
    //         OutputPort<int>("precision"),
    //         OutputPort<bool>("longRotationAlignResult"),
    //         // OutputPort<TrajectoryPose>("tp", output_state);
    //     };
    // }

    void RotateToMission::onHalted()
    {

    }

    BT::NodeStatus RotateToMission::onStart()
    {
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus RotateToMission::onRunning()
    {
        AiDetectionArray arr;

        // We get the detected object by the AI
        getInput("detections", arr);

        if (arr.detection_array.empty())
        {
            // publish safe defaults
            std::cout <<"In Alignement Detection array empty"<< std::endl;

            return BT::NodeStatus::FAILURE;
        }

        // Assumption: array is pre-filtered for the object of interest → use first detection
        const AiDetection& det = arr.detection_array.front(); // A verifier si on peux renvoyer le plus proche a la place et non la premiere detection

        // getInput("alignement_by_translation", mode);
        AlignResult alignres;
        alignres.align_type = SEARCH_MISSION;
        alignres.distance = det.distance;
        alignres.rad_x = GetCenterImageAngleX(det);

        // AlignResult res_y = computeAlignmentY(det,mode);
        std::cout <<"AlignResult.align_type : "<< alignres.align_type << std::endl;
        std::cout <<"AlignResult.distance : "<< alignres.distance << std::endl;
        std::cout <<"AlignResult.rad_x : "<< alignres.rad_x << std::endl;

        // Publish outputs Je modifirais pour envoyer une trajectoire tout simplement. Ce que la fonction ferais tout simplement de calculer une trajectoire par rapport au donnee de l'IA
        // setOutput("bearing_rad", res.bearing_rad);
        // setOutput("norm_x", res.norm_x);
        // std::cout <<"center x pixel norm : "<< res.bearing_rad << std::endl;
        // setOutput("has_metric", res.has_metric ? 1 : 0);
        
        // if (res.has_metric)
        // {
        //     setOutput("lateral_m", res.lateral_m);
        //     prev_lateral_ = res.lateral_m;
        // }

        // prev_bearing_ = res.bearing_rad;
        // std::cout <<"center x pixel norm : "<< res.bearing_rad << std::endl;

        // prev_normx_ = res.norm_x;

        // std::cout <<"res has metric : "<< res.has_metric << std::endl;
        if (alignres.align_type != NOT_CHOOSEN_YET){
            
            TrajectoryPose t;

            // t.positionY = 0.0;
            // if(mode==1 or camera==false){
            //     t.positionY = res.lateral_m;
            //     std::cout <<"translation y : "<< res.lateral_m << std::endl;
            //     }
            // // setOutput("positionY", positionY);
            // t.positionX = 0.0;
            // if (camera==false){
            //     t.positionY = res_y.lateral_m;
            //     std::cout <<"translation x : "<< t.positionY << std::endl;
            // }
            // // setOutput("positionX", positionX);
            // t.positionZ = 0.0;
            // // setOutput("positionZ", positionZ);
            // t.orientationX = 0.0;
            // // setOutput("orientationX", orientationX);

            // t.orientationY = 0;
            // // setOutput("orientationY", orientationY);
            // t.orientationZ = 0;
            // std::cout << "position en x: "<<t.positionX;
            // if (!mode){
            //     t.orientationZ = res.bearing_rad / M_PI * 180;
            //     std::cout <<"orientation z : "<< res.bearing_rad << std::endl;

            // }
            // // int frame = 1;
            // t.frame = 1;
            // // setOutput("frame", frame);
            // // int speed = 2;
            // // setOutput("speed", speed);
            // t.speed = 2;
            // t.precision = 0;
            // t.long_rotation=false;
            // setOutput("precision", precision);
            // bool long_rotation = false;
            // setOutput("longRotation", long_rotation);

            // struct TrajectoryPose
            // {
            //     float positionX;
            //     float positionY;
            //     float positionZ;
            //     float orientationX;
            //     float orientationY;
            //     float orientationZ;
            //     int frame;
            //     int speed;
            //     int precision;
            //     bool long_rotation;
            // };

            t = ComputeTrajectory(alignres);
            // t.positionY = 0.0;
            // t.positionZ = 0.0;
            // t.orientationX = 0.0;
            // t.orientationY = 0.0;
            // t.orientationZ = AlignResult.bearing_rad;
            // t.frame = 1.0;
            // t.speed = 2;
            // t.presision = 0;



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
            Trajectory traj = getInput<Trajectory>("trajectory").value();
            traj.trajectory.push_back(t);
            setOutput<Trajectory>("trajectory", traj);
            // setOutput<float>("TranslationX_CamBottom",t.positionX);
            // setOutput<float>("TranslationY",t.positionY);
            // setOutput<float>("RotationZ",t.orientationZ);
        }

        // SUCCESS if metric lateral is available; otherwise RUNNING so parent can fall back to bearing-only logic
        return (alignres.align_type == NOT_CHOOSEN_YET) ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation