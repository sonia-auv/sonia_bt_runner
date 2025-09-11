#include "sonia_bt_runner/navigation/MainAlignment.hpp"
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

float GetCenterImageAngleX(const AiDetection& det)
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

float GetCenterImageAngleY(const AiDetection& det)
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

Trajectory ComputeTrajectory(const AlignResult res)
{
    
}

