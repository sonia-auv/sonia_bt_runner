#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"

// Constant list for droppers and torpidoes3
const float CAMERA_TO_DROPPER_OFFSET_X = 0.12;
const float CAMERA_TO_DROPPER_OFFSET_Y = 0.18;
const float CAMERA_TO_TORPIDO_PEPPER_OFFSET_X = -0.072;
const float CAMERA_TO_TORPIDO_SALT_OFFSET_X = 0.11;
const float CAMERA_TO_TORPEDO_OFFSET_Y = -0.10;


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