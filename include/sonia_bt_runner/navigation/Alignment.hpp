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
    static constexpr int width = 672; 
    static constexpr int height = 376;
    static constexpr float fovx = 30.650667*2;              // px (typical)
    static constexpr float fov_y = 85;              // px (typical)
    static constexpr float cx = (width - 1) * 0.5f;  // ~ image center ()
    static constexpr float c_y = (height - 1) * 0.5f;  // ~ image center ()
};

// Results of alignment computation
struct AlignResult
{
    bool has_metric = false;                                    // true if lateral_m valid
    float lateral_m = std::numeric_limits<float>::quiet_NaN();  // +right, -left
    float bearing_rad = 0.0f;                                   // yaw to target
    float norm_x = 0.0f;                                        // [-1..1] approx
    bool in_interval=false;                                     // check if near the target
};

// Helpers (declared here, defined in .cpp)
float bboxCenterX(const AiDetection& d);
AlignResult computeAlignmentHD720(const AiDetection& det, bool alignement_by_trans);
float bboxCenterY(const AiDetection& d);
AlignResult computeAlignmentY(const AiDetection& det, bool alignement_by_trans);

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

    class Alignment : public BT::StatefulActionNode
    {
        public:
            Alignment(const std::string &name, const BT::NodeConfig &config,std::shared_ptr<rclcpp::Node> node);
            ~Alignment()=default;
            static BT::PortsList providedPorts()
            {
                return {
                        // Inputs
                    BT::InputPort<AiDetectionArray>("detections"),
                    BT::InputPort<bool>("alignement_by_translation"),//
                    BT::InputPort<bool>("Camera_front"),

                    // Outputs
                    BT::BidirectionalPort<Trajectory>("traj"),
                    BT::OutputPort<float>("TranslationX_CamBottom"),
                    BT::OutputPort<float>("TranslationY"),
                    BT::OutputPort<float>("RotationZ"),
                    // BT::OutputPort<float>("positionX"),
                    // BT::OutputPort<float>("positionY"),
                    // BT::OutputPort<float>("positionZ"),
                    // BT::OutputPort<float>("orientationX"),
                    // BT::OutputPort<float>("orientationY"),
                    // BT::OutputPort<float>("orientationZ"),
                    // BT::OutputPort<int>("frame"),
                    // BT::OutputPort<int>("speed"),
                    // BT::OutputPort<int>("precision"),
                    // BT::OutputPort<bool>("longRotation"),
                    // OutputPort<TrajectoryPose>("tp", output_state),
                };
            }

            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;


        // Alignment(const std::string& name, const BT::NodeConfig& cfg);

        // static BT::PortsList providedPorts();

        // BT::NodeStatus tick() override;

        private:
            std::shared_ptr<rclcpp::Node> ros_node;


            BT::Expected<AiDetectionArray> arr;
            BT::Expected<bool> mode;
            BT::Expected<bool>camera;
            // Previous outputs for optional temporal smoothing
            float prev_lateral_ = std::numeric_limits<float>::quiet_NaN();
            float prev_bearing_ = std::numeric_limits<float>::quiet_NaN();
            float prev_normx_ = std::numeric_limits<float>::quiet_NaN();


    };

}  // namespace navigation
