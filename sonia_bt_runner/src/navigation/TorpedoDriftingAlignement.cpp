#include "sonia_bt_runner/navigation/TorpedoDriftingAlignement.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>

namespace navigation
{

    TorpedoDriftingAlignement::TorpedoDriftingAlignement(const std::string &name, const BT::NodeConfig &config,std::shared_ptr<rclcpp::Node> node)
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

    void TorpedoDriftingAlignement::onHalted()
    {

    }

    BT::NodeStatus TorpedoDriftingAlignement::onStart()
    {
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus TorpedoDriftingAlignement::onRunning()
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
        
        // We compute a rotation to make the sub rotate at 90 degrees to the target
        TrajectoryPose t1;
        t1.positionX = 0.0;
        t1.positionY =0.0;
        t1.positionZ = 0.0;
        t1.orientationX = 0.0;
        t1.orientationY = 0.0;
        t1.orientationZ = -det.angle_teta;
        t1.frame = 1;
        t1.speed = 0;
        t1.precision = 0;
        t1.long_rotation = false;

        std::cout << "position en x: "<<t1.positionX<< std::endl;
        std::cout<< "position en y: "<<t1.positionY<< std::endl;
        std::cout << "position en z: "<<t1.positionZ<< std::endl;
        std::cout << "Orientation en x: "<<t1.orientationX<< std::endl;
        std::cout << "Orientation en y: "<<t1.orientationY<< std::endl;
        std::cout << "Orientation en z: "<<t1.orientationZ<< std::endl;
        std::cout << "frame: "<<t1.frame<< std::endl;
        std::cout << "speed: "<<t1.speed<< std::endl;
        std::cout << "precision: "<<t1.precision<< std::endl;
        std::cout << "long rotation: "<<t1.long_rotation<< std::endl;

        // We compute a translation in Y to go in front of the target
        TrajectoryPose t2;
        t2.positionX = 0.0;
        t2.positionY = det.distance_teta;
        t2.positionZ = 0.0;
        t2.orientationX = 0.0;
        t2.orientationY = 0.0;
        t2.orientationZ = 0.0;
        t2.frame = 1;
        t2.speed = 0;
        t2.precision = 0;
        t2.long_rotation = false;

        std::cout << "position en x: "<<t2.positionX<< std::endl;
        std::cout<< "position en y: "<<t2.positionY<< std::endl;
        std::cout << "position en z: "<<t2.positionZ<< std::endl;
        std::cout << "Orientation en x: "<<t2.orientationX<< std::endl;
        std::cout << "Orientation en y: "<<t2.orientationY<< std::endl;
        std::cout << "Orientation en z: "<<t2.orientationZ<< std::endl;
        std::cout << "frame: "<<t2.frame<< std::endl;
        std::cout << "speed: "<<t2.speed<< std::endl;
        std::cout << "precision: "<<t2.precision<< std::endl;
        std::cout << "long rotation: "<<t2.long_rotation<< std::endl;

        // We append the two new trajectory points to the sending trajectory
        Trajectory traj = getInput<Trajectory>("Trajectory").value();
        traj.trajectory.push_back(t1);
        traj.trajectory.push_back(t2);
        setOutput<Trajectory>("Trajectory", traj);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation