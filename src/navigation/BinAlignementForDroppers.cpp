#include "sonia_bt_runner/navigation/BinAlignementForDroppers.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>

namespace navigation
{

    BinAlignementForDroppers::BinAlignementForDroppers(const std::string &name, const BT::NodeConfig &config,std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), ros_node(node), valid(0), _time_launch(std::chrono::system_clock::now())
    {
        
    }
    static BT::PortsList providedPorts()
    {
        return {
            // Inputs
            BT::InputPort<AiDetectionArray>("Detections"),

            // Outputs
            BT::BidirectionalPort<Trajectory>("Trajectory"),
        };
    }

    void BinAlignementForDroppers::onHalted()
    {

    }

    BT::NodeStatus BinAlignementForDroppers::onStart()
    {
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus BinAlignementForDroppers::onRunning()
    {
        AiDetectionArray arr;

        // We get the detected object by the AI
        getInput("Detections", arr);

        if (arr.detection_array.empty())
        {
            // publish safe defaults
            std::cout <<"In Alignement Detection array empty"<< std::endl;

            return BT::NodeStatus::FAILURE;
        }

        // Assumption: array is pre-filtered for the object of interest → use first detection
        const AiDetection& det = arr.detection_array.front(); // A verifier si on peux renvoyer le plus proche a la place et non la premiere detection
        
        // We compute the trajectory to do
        TrajectoryPose t;

        // t.positionX = -det.distance_beta - CAMERA_TO_DROPPER_OFFSET_X;
        t.positionY = det.distance_teta - CAMERA_TO_DROPPER_OFFSET_Y;
        t.positionZ = 0.0;
        t.orientationX = 0.0;
        t.orientationY = 0.0;
        t.orientationZ = 0.0;
        t.frame = 1;
        t.speed = 0;
        t.long_rotation = false;

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

        // We append the new trajectory point to the sending trajectory
        Trajectory traj = getInput<Trajectory>("Trajectory").value();
        traj.trajectory.push_back(t);
        setOutput<Trajectory>("Trajectory", traj);

        // SUCCESS if metric lateral is available; otherwise RUNNING so parent can fall back to bearing-only logic
        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation