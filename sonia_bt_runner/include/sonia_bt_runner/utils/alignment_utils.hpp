#pragma once
#include <cmath>
#include <limits>
#include "sonia_bt_runner/utils/AiDetection.hpp"

struct CameraInfoZedMiniHD720 {
    static constexpr int   width   = 1280;
    static constexpr int   height  = 720;
    static constexpr float fx      = 736.0f;   // typical ZED Mini @ 720p
    static constexpr float cx      = (width - 1) * 0.5f; // ~ image center
};
