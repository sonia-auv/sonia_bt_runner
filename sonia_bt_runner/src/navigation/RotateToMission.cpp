#include "sonia_bt_runner/navigation/RotateToMission.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>

namespace navigation
{

    RotateToMission::RotateToMission(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
    {

    }

    BT::NodeStatus RotateToMission::tick()
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

        TrajectoryPose t;

        // We set the trajectory of the submarine to rotate on itself
        t.positionX = 0.0; // We don't move on the x axis
        t.positionY = 0.0; // We don't move on the x and y axis
        t.positionZ = 0.0; // We don't move on the z axis
        t.orientationX = 0.0; // We don't turn around the x axis
        t.orientationY = 0.0; // We don't turn around the y axis
        t.orientationZ = -det.angle_alpha;  // We turn around the z axis
        t.frame = 1;                        // 0: relative to current position, 1
        t.speed = 0;                        // Speed of the rotation
        t.precision = 0;                    // Precision of the rotation
        t.long_rotation = true;             // true: take the long way around, false:

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
        Trajectory traj = getInput<Trajectory>("Trajectory").value();
        traj.trajectory.push_back(t);
        setOutput<Trajectory>("Trajectory", traj);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation