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

inline float bboxCenterX(const AiDetection& d)
{
    return 0.25f * (d.top_left_x + d.top_right_x +
                    d.bottom_left_x + d.bottom_right_x);
}

struct AlignResult {
    bool  has_metric   = false;
    float lateral_m    = std::numeric_limits<float>::quiet_NaN();
    float bearing_rad  = 0.0f;
    float norm_x       = 0.0f;
};

inline AlignResult computeAlignmentHD720(const AiDetection& det,
                                         bool coords_are_normalized = false)
{
    AlignResult out;
    float u = bboxCenterX(det);
    float u_px = coords_are_normalized ? (u * CameraInfoZedMiniHD720::width) : u;

    out.norm_x = 2.0f * (u_px - CameraInfoZedMiniHD720::cx) /
                 std::max(1, CameraInfoZedMiniHD720::width);

    out.bearing_rad = std::atan((u_px - CameraInfoZedMiniHD720::cx) /
                                CameraInfoZedMiniHD720::fx);

    if (std::isfinite(det.distance) && det.distance > 0.0f) {
        out.lateral_m = det.distance * (u_px - CameraInfoZedMiniHD720::cx) /
                        CameraInfoZedMiniHD720::fx;
        out.has_metric = std::isfinite(out.lateral_m);
    }
    return out;
}
