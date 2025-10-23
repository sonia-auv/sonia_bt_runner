#include "sonia_bt_runner/navigation/MoveToObject.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>

namespace navigation
{

    MoveToObject::MoveToObject(const std::string &name, const BT::NodeConfig &config)
       : BT::SyncActionNode(name, config)
    {

    }
    BT::NodeStatus MoveToObject::tick()
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

        TrajectoryPose t1;
        // We set the trajectory of the submarine to rotate on itself to face the object
        t1.positionX = 0.0; // We don't move on the x axis
        t1.positionY = 0.0; // We don't move on the x and y axis
        t1.positionZ = 0.0; // We don't move on the z axis
        t1.orientationX = 0.0; // We don't turn around the x axis
        t1.orientationY = 0.0; // We don't turn around the y axis
        t1.orientationZ = -det.angle_alpha;  // We turn around the z axis
        t1.frame = 1;                        // 0: relative to current position, 1
        t1.speed = 0;                        // Speed of the rotation
        t1.precision = 0;                    // Precision of the rotation
        t1.long_rotation = true;             // true: take the long way around, false:

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

        // We move forward to go to the object
        TrajectoryPose t2;
        t2.positionX = det.distance; // We go to the object
        t2.positionY = 0.0; // We don't move lateraly
        t2.positionZ = 0.0; // We don't move up or down
        t2.orientationX = 0.0; // We don't rotate on the X axes
        t2.orientationY = 0.0; // We don't rotate on the Y axes
        t2.orientationZ = 0.0; // We rotate on the Z axes to face
        t2.frame = 1.0; // We use the frame in meter
        t2.speed = 0; // We use the default speed of the motion controller
        t2.precision = 0; // We use the default precision of the motion controller
        t2.long_rotation = false; // We don't use long rotation

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

        // We append the new pose to the trajectory
        Trajectory traj = getInput<Trajectory>("Trajectory").value();
        traj.trajectory.push_back(t1);
        traj.trajectory.push_back(t2);
        setOutput<Trajectory>("Trajectory", traj);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace navigation