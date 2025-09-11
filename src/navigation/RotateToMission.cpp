#include "sonia_bt_runner/navigation/RotateToMission.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>

namespace navigation
{
    class RotateToMission : public BT::StatefulActionNode, public MainAlignment

        public:
            RotateToMission(const std::string &name, const BT::NodeConfig &config,std::shared_ptr<rclcpp::Node> node){}
            ~RotateToMission()=default;
            static BT::PortsList providedPorts()
            {
                return {
                    // Inputs
                    BT::InputPort<AiDetectionArray>("detections"),

                    // Outputs
                    BT::BidirectionalPort<Trajectory>("traj"),
                };
            }

            BT::NodeStatus onStart() override
            {
                
            }
            BT::NodeStatus onRunning() override
            {

            }
            void onHalted()
            {

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

    void SearchMissionAlignment::onHalted()
    {

    }

    BT::NodeStatus SearchMissionAlignment::onStart()
    {
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus SearchMissionAlignment::onRunning()
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

        getInput("alignement_by_translation", mode);
        AlignResult res = AlignmentToDetecedMission(det);
        // AlignResult res_y = computeAlignmentY(det,mode);
        std::cout <<"center x pixel norm : "<< res.bearing_rad << std::endl;

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

        std::cout <<"res has metric : "<< res.has_metric << std::endl;
        if (res.has_metric){
            
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

            t.positionX = 0.0;
            t.positionY = 0.0;
            t.positionZ = 0.0;
            t.orientationX = 0.0;
            t.orientationY = 0.0;
            t.orientationZ = AlignResult.bearing_rad;
            t.frame = 1.0;
            t.speed = 2;
            t.presision = 0;



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
            // setOutput<float>("TranslationX_CamBottom",t.positionX);
            // setOutput<float>("TranslationY",t.positionY);
            // setOutput<float>("RotationZ",t.orientationZ);
        }

        // SUCCESS if metric lateral is available; otherwise RUNNING so parent can fall back to bearing-only logic
        return res.has_metric ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }

}  // namespace navigation