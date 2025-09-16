#include "sonia_bt_runner/navigation/MainAlignment.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <math.h>

namespace navigation{

    // ----------------- Helper implementations -----------------

    float MainAlignment::bboxCenterX(const AiDetection& d)
    {
        // Average all four corners (robust to minor skew)
        return 0.25f * (d.top_left_x + d.top_right_x + d.bottom_left_x + d.bottom_right_x);
    }

    float MainAlignment::bboxCenterY(const AiDetection& d)
    {
        // Average all four corners (robust to minor skew)
        return 0.25f * (d.top_left_y + d.top_right_y + d.bottom_left_y + d.bottom_right_y);
    }

    float MainAlignment::GetCenterImageAngleX(const AiDetection& det)
    {
        // We get the x distance between the left side of the image and the center of the object.
        float u_px = bboxCenterX(det);
        std::cout <<"center x pixel : "<< u_px << std::endl;
        std::cout <<"center dist x pixel norm : "<< (u_px - CameraInfoZedMiniHD720::cx)  << std::endl;

        // We get the adjacent value of the rectangle triangle with the camera angle (fovx) and the adjacent value width.
        float fx=(CameraInfoZedMiniHD720::width/2)/(tan(CameraInfoZedMiniHD720::fovx*M_PI/180/2));

        // We get the angle between the normal of the cam and the direction to the detected object.  
        return atan((u_px - CameraInfoZedMiniHD720::cx) /(fx));
    }

    float MainAlignment::GetCenterImageAngleY(const AiDetection& det)
    {
        // We get the y distance between the top and the center of the detected object
        float u_py = bboxCenterY(det);
        std::cout <<"center x pixel : "<< u_py << std::endl;
        std::cout <<"center dist x pixel norm : "<< (u_py - CameraInfoZedMiniHD720::cx)  << std::endl;

        // We get the adjacent value of the rectangle triangle with the camera angle (fovx) and the adjacent value width.
        float fx=(CameraInfoZedMiniHD720::width/2)/(tan(CameraInfoZedMiniHD720::fovx*M_PI/180/2));

        // We get the angle between the normal of the cam and the direction to the detected object.  
        return atan((u_py - CameraInfoZedMiniHD720::cx) /(fx));
    }
    


    TrajectoryPose MainAlignment::ComputeTrajectory(const AlignResult res)
    {
        TrajectoryPose compute_trajectory;

        compute_trajectory.positionX = 0.0;
        compute_trajectory.positionY = 0.0;
        compute_trajectory.positionZ = 0.0;
        compute_trajectory.orientationX = 0.0;
        compute_trajectory.orientationY = 0.0;
        compute_trajectory.orientationZ = 0.0;
        compute_trajectory.frame = 1;
        compute_trajectory.speed = 2;
        compute_trajectory.precision = 0;
        compute_trajectory.long_rotation=false;
        switch (res.align_type)
        {
            case NOT_CHOOSEN_YET:
                std::cout << "No alignment type chosen yet." << std::endl;
                return compute_trajectory; // Return an empty trajectory
                break;
            case SEARCH_MISSION:
            case GATE_ALIGN_TO_OBJECT:
                compute_trajectory.orientationZ = res.rad_x;
                break;
            case GATE_APPROACH:
                compute_trajectory.positionX = res.distance;
                break;
            case TORPIDOES_DRIFTING:
                break;
            case TORPIDOES_TARGET:
                break;
            case BIN_FRONT:
                compute_trajectory.positionX = res.distance * cos(res.rad_y);
                break;
            case BIN_BOTTOM:
                compute_trajectory.positionX = res.distance * sin(res.rad_x);
                compute_trajectory.positionY = res.distance * sin(res.rad_y);
                break;
            case TABLE_APPROACH:
                break;
            case TABLE_GRAB:
                break;
        }

        
        std::cout << "position en x: "<<compute_trajectory.positionX<< std::endl;
        std::cout<< "position en y: "<<compute_trajectory.positionY<< std::endl;
        std::cout << "position en z: "<<compute_trajectory.positionZ<< std::endl;
        std::cout << "Orientation en x: "<<compute_trajectory.orientationX<< std::endl;
        std::cout << "Orientation en y: "<<compute_trajectory.orientationY<< std::endl;
        std::cout << "Orientation en z: "<<compute_trajectory.orientationZ<< std::endl;
        std::cout << "frame: "<<compute_trajectory.frame<< std::endl;
        std::cout << "speed: "<<compute_trajectory.speed<< std::endl;
        std::cout << "precision: "<<compute_trajectory.precision<< std::endl;
        std::cout << "long rotation: "<<compute_trajectory.long_rotation<< std::endl;
        return compute_trajectory;
    }

}

