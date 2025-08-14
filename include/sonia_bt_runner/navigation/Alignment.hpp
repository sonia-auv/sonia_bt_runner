#pragma once

#include <limits>

#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_bt_runner/utils/AiDetectionArray.hpp"

// Fixed intrinsics for ZED Mini @ HD720 (single-eye image: 1280x720)
struct CameraInfoZedMiniHD720
{
    static constexpr int width = 1280;
    static constexpr int height = 720;
    static constexpr float fx = 736.0f;              // px (typical)
    static constexpr float cx = (width - 1) * 0.5f;  // ~ image center (639.5)
};

// Results of alignment computation
struct AlignResult
{
    bool has_metric = false;                                    // true if lateral_m valid
    float lateral_m = std::numeric_limits<float>::quiet_NaN();  // +right, -left
    float bearing_rad = 0.0f;                                   // yaw to target
    float norm_x = 0.0f;                                        // [-1..1] approx
};

// Helpers (declared here, defined in .cpp)
float bboxCenterX(const AiDetection& d);
AlignResult computeAlignmentHD720(const AiDetection& det, bool coords_are_normalized);

// BehaviorTree node: Alignment
// Inputs:
//   - detections (AiDetectionArray): pre-filtered; we take the first element
//   - normalized_coords (bool, default false): true if detection xy in [0..1]
//   - alpha (float, default -1): optional smoothing factor in (0,1]; <=0 disables
// Outputs:
//   - lateral_m (float): meters (+right, -left) if distance valid
//   - bearing_rad (float): radians (useful even without distance)
//   - norm_x (float): [-1,1] normalized lateral
//   - has_metric (int): 1 if lateral_m valid, else 0
namespace navigation
{

    class Alignment : public BT::SyncActionNode
    {
        public:
        Alignment(const std::string& name, const BT::NodeConfig& cfg);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

        private:
        // Previous outputs for optional temporal smoothing
        float prev_lateral_ = std::numeric_limits<float>::quiet_NaN();
        float prev_bearing_ = std::numeric_limits<float>::quiet_NaN();
        float prev_normx_ = std::numeric_limits<float>::quiet_NaN();
    };

}  // namespace navigation
