#include "sonia_bt_runner/navigation/MoveToMission.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>

namespace navigation
{

    MoveToMission::MoveToMission(const std::string &name, const BT::NodeConfig &config,std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), ros_node(node), valid(0), _time_launch(std::chrono::system_clock::now())
    {

    }
    static BT::PortsList providedPorts()
    {
        return {
            // Inputs
            BT::InputPort<AiDetectionArray>("detections"),

            // Outputs
            BT::BidirectionalPort<Trajectory>("trajectory"),
        };
    }

    void MoveToMission::onHalted()
    {

    }

    BT::NodeStatus MoveToMission::onStart()
    {
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus MoveToMission::onRunning()
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

        // We compute the trajectory with the AI detection
        TrajectoryPose t;
        t.positionX = det.distance; // We go to the object
        t.positionY = 0.0; // We don't move lateraly
        t.positionZ = 0.0; // We don't move up or down
        t.orientationX = 0.0; // We don't rotate on the X axes
        t.orientationY = 0.0; // We don't rotate on the Y axes
        t.orientationZ = det.angle_teta; // We rotate on the Z axes to face
        t.frame = 1.0; // We use the frame in meter
        t.speed = 0; // We use the default speed of the motion controller
        t.precision = 0; // We use the default precision of the motion controller
        t.long_rotation = false; // We don't use long rotation

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

        // We append the new pose to the trajectory
        Trajectory traj = getInput<Trajectory>("trajectory").value();
        traj.trajectory.push_back(t);
        setOutput<Trajectory>("trajectory", traj);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation