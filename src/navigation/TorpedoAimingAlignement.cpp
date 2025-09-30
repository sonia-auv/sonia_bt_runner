#include "sonia_bt_runner/navigation/TorpedoAimingAlignement.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>

namespace navigation
{

    TorpedoAimingAlignement::TorpedoAimingAlignement(const std::string &name, const BT::NodeConfig &config,std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), ros_node(node), valid(0), _time_launch(std::chrono::system_clock::now())
    {

    }
    static BT::PortsList providedPorts()
    {
        return {
            // Inputs
            BT::InputPort<AiDetectionArray>("Detections"),
            BT::InputPort<std::string>("LaunchingSide"),

            // Outputs
            BT::BidirectionalPort<Trajectory>("Trajectory"),
        };
    }

    void TorpedoAimingAlignement::onHalted()
    {

    }

    BT::NodeStatus TorpedoAimingAlignement::onStart()
    {
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus TorpedoAimingAlignement::onRunning()
    {
        AiDetectionArray arr;
        std::string launching_side;

        // We get the detected object by the AI
        getInput("Detections", arr);
        getInput("LaunchingSide", launching_side);

        if (arr.detection_array.empty())
        {
            // publish safe defaults
            std::cout <<"In Alignement Detection array empty"<< std::endl;

            return BT::NodeStatus::FAILURE;
        }

        // Assumption: array is pre-filtered for the object of interest → use first detection
        const AiDetection& det = arr.detection_array.front(); // A verifier si on peux renvoyer le plus proche a la place et non la premiere detection
        
        TrajectoryPose t1;
        t.positionX = 0.0; // We don't move on the x axis
        if (launching_side == "portside")
        {
            t.positionY = det.distance_teta - CAMERA_TO_TORPIDO_PEPPER_OFFSET_X; // We move on the x axis
        }
        else
        {
            t.positionY = det.distance_teta - CAMERA_TO_TORPIDO_SALT_OFFSET_X; // We move on the x axis
        }
        // t.positionZ = det.distance_beta - CAMERA_TO_TORPEDO_OFFSET_Y; // We move on the z axis
        t.orientationX = 0.0;       // We don't rotate on the x axis
        t.orientationY = 0.0;       // We don't rotate on the y axis
        t.orientationZ = 0.0;       // We don't rotate on the z axis
        t.frame = 1;                // We use the relative positionning
        t.speed = 0;
        t.precision = 0;            // We don't care about the precision
        t.long_rotation = false;    // We don't care about the long rotation

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
        Trajectory traj = getInput<Trajectory>("Trajectory").value();
        traj.trajectory.push_back(t);
        setOutput<Trajectory>("Trajectory", traj);

        // SUCCESS if metric lateral is available; otherwise RUNNING so parent can fall back to bearing-only logic
        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation