#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"


// Fixed intrinsics for ZED Mini @ HD720 (single-eye image: 1280x720)
struct CameraInfoZedMiniHD720
{
    // dimension image for VGA
    static constexpr int width = 672;                           // Width of the image
    static constexpr int height = 376;                          // Height of the image
    static constexpr float fovx = 30.650667*2;                  // Vision angle on x axes
    static constexpr float fov_y = 85;                          // Vision angle on y axes
    static constexpr float cx = (width - 1) * 0.5f;             // X center position of the image
    static constexpr float c_y = (height - 1) * 0.5f;           // Y center posiion of the image
};

// Struct to define the Align result type
enum AlignmentType{
    NOT_CHOOSEN_YET = 0,
    SEARCH_MISSION = 1,
    MOVE_TO_MISSION = 2
    TORPIDOES_DRIFTING = 3,
    TORPIDOES_TARGET = 4,
    BIN_FRONT = 5,
    BIN_BOTTOM = 6,
    TABLE_GRAB = 7
};

// Results of alignment computation
struct AlignResult
{   
    float rad_x = 0.0f;             // x angle
    float rad_y = 0.0f;             // y angle
    float distance = 0.0f;          // The distance of the object
    AlignmentType align_type;       // The type of alignement
};

namespace navigation{
    class MainAlignment{
        public:
            /*
            * Fonction bboxCenterX
            *  Description  :  Use to compute the X center value of the detected image on the camera
            * 
            *  Input Param  :  det
            *                      Detection array gather from the AI
            * 
            * OutPut Param :  The Y position value of the center of the detected image
            */
            float bboxCenterX(const AiDetection& det);

            /*
            * Fonction bboxCenterY
            *  Description :  Use to compute the Y center value of the detected image on the camera
            * 
            *  Input Param :  det
            *                      Detection array gather from the AI
            * 
            *  Return Value:  The Y position value of the center of the detected image
            */
            float bboxCenterY(const AiDetection& det);

            /*
            * Fonction GetCenterImageAngleX
            *  Description :  Use to compute the Y center value of the detected image on the camera
            * 
            *  Input Param :  det
            *                      Detection array gather from the AI
            * 
            *  Return Value:  The Y position value of the center of the detected image
            */
            float GetCenterImageAngleX(const AiDetection& det);

            /*
            * Fonction GetCenterImageAngleY
            *  Description :  Use to compute the Y center value of the detected image on the camera
            * 
            *  Input Param :  det
            *                      Detection array gather from the AI
            * 
            *  Return Value:  The Y position value of the center of the detected image
            */
            float GetCenterImageAngleY(const AiDetection& det);

            /*
            * Fonction ComputeTrajectory
            *  Description :  Use to compute the Y center value of the detected image on the camera
            * 
            *  Input Param :  d
            *                      Detection array gather from the AI
            * 
            *  Return Value:  The Y position value of the center of the detected image
            */
            TrajectoryPose ComputeTrajectory(const AlignResult res);
    };
}
