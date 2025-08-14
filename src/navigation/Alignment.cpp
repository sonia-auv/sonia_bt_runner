#include "sonia_bt_runner/navigation/Alignment.hpp"

#include <algorithm>
#include <cmath>

using namespace BT;

// ----------------- Helper implementations -----------------

float bboxCenterX(const AiDetection& d)
{
    // Average all four corners (robust to minor skew)
    return 0.25f * (d.top_left_x + d.top_right_x + d.bottom_left_x + d.bottom_right_x);
}

AlignResult computeAlignmentHD720(const AiDetection& det, bool coords_are_normalized)
{
    AlignResult out;

    // 1) detection center x in pixels
    float u = bboxCenterX(det);
    float u_px = coords_are_normalized ? (u * CameraInfoZedMiniHD720::width) : u;

    // 2) normalized lateral in [-1, 1] (no meters; control hint/UI)
    out.norm_x = 2.0f * (u_px - CameraInfoZedMiniHD720::cx) / std::max(1, CameraInfoZedMiniHD720::width);

    // 3) bearing (radians) using fx/cx (positive = target to the right)
    out.bearing_rad = std::atan((u_px - CameraInfoZedMiniHD720::cx) / CameraInfoZedMiniHD720::fx);

    // 4) metric lateral if distance is valid: x = Z * (u - cx) / fx
    if (std::isfinite(det.distance) && det.distance > 0.0f)
    {
        out.lateral_m = det.distance * (u_px - CameraInfoZedMiniHD720::cx) / CameraInfoZedMiniHD720::fx;
        out.has_metric = std::isfinite(out.lateral_m);
    }
    return out;
}

// ----------------- Alignment node -----------------

namespace navigation
{

    Alignment::Alignment(const std::string& name, const BT::NodeConfig& cfg) : BT::SyncActionNode(name, cfg) {}

    BT::PortsList Alignment::providedPorts()
    {
        return {
            // Inputs
            InputPort<AiDetectionArray>("detections"),
            InputPort<bool>("normalized_coords"),
            InputPort<float>("alpha"),

            // Outputs
            OutputPort<float>("lateral_m"),
            OutputPort<float>("bearing_rad"),
            OutputPort<float>("norm_x"),
            OutputPort<int>("has_metric"),
        };
    }

    BT::NodeStatus Alignment::tick()
    {
        AiDetectionArray arr;
        if (!getInput("detections", arr) || arr.detection_array.empty())
        {
            // publish safe defaults
            setOutput("bearing_rad", 0.0f);
            setOutput("norm_x", 0.0f);
            setOutput("has_metric", 0);
            return BT::NodeStatus::FAILURE;
        }

        bool normalized = false;
        getInput("normalized_coords", normalized);
        float alpha = -1.0f;
        getInput("alpha", alpha);
        const bool do_smooth = (alpha > 0.0f && alpha <= 1.0f);

        // Assumption: array is pre-filtered for the object of interest → use first detection
        const AiDetection& det = arr.detection_array.front();
        AlignResult res = computeAlignmentHD720(det, normalized);

        // Optional temporal smoothing (IIR) for stability
        if (do_smooth)
        {
            if (std::isfinite(prev_bearing_))
                res.bearing_rad = alpha * res.bearing_rad + (1.0f - alpha) * prev_bearing_;
            if (std::isfinite(prev_normx_)) res.norm_x = alpha * res.norm_x + (1.0f - alpha) * prev_normx_;
            if (res.has_metric && std::isfinite(prev_lateral_))
                res.lateral_m = alpha * res.lateral_m + (1.0f - alpha) * prev_lateral_;
        }

        // Publish outputs
        setOutput("bearing_rad", res.bearing_rad);
        setOutput("norm_x", res.norm_x);
        setOutput("has_metric", res.has_metric ? 1 : 0);
        if (res.has_metric)
        {
            setOutput("lateral_m", res.lateral_m);
            prev_lateral_ = res.lateral_m;
        }

        prev_bearing_ = res.bearing_rad;
        prev_normx_ = res.norm_x;

        // SUCCESS if metric lateral is available; otherwise RUNNING so parent can fall back to bearing-only logic
        return res.has_metric ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }

}  // namespace navigation
