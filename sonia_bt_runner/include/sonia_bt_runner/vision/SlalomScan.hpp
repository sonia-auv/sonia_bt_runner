#pragma once

#include "sonia_bt_runner/vision/AbstractAiFilter.hpp"
#include "sonia_bt_runner/utils/SlalomPointArray.hpp"

#define SLALOM_SCAN_ACTUAL_ANGLE "Actual_angle"
#define SLALOM_SCAN_ACTUAL_ANGLE_PARAMS SLALOM_SCAN_ACTUAL_ANGLE, "The actual absolute angle of the sub"
#define SLALOM_SCAN_ACTUAL_ANGLE_TYPE float

#define SLALOM_SCAN_INVERT_CAMERA_ANGLE "Invert_camera_angle"
#define SLALOM_SCAN_INVERT_CAMERA_ANGLE_PARAMS SLALOM_SCAN_INVERT_CAMERA_ANGLE, false, "Flip the sign convention of the camera angle if the computed points are mirrored"
#define SLALOM_SCAN_INVERT_CAMERA_ANGLE_TYPE bool

#define SLALOM_SCAN_POINTS "Points"
#define SLALOM_SCAN_POINTS_PARAMS SLALOM_SCAN_POINTS, "The accumulated slalom points, relative to the sub position during the scan"
#define SLALOM_SCAN_POINTS_TYPE SlalomPointArray

namespace vision{
    class SlalomScan: public AbstractAiFilter {
        public:
            SlalomScan(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~SlalomScan() override = default;
            static BT::PortsList providedPorts()
            {
                auto default_ports = AbstractAiFilter::providedPorts();
                BT::PortsList new_ports = {
                    // Inputs
                    BT::InputPort<SLALOM_SCAN_ACTUAL_ANGLE_TYPE>(SLALOM_SCAN_ACTUAL_ANGLE_PARAMS),
                    BT::InputPort<SLALOM_SCAN_INVERT_CAMERA_ANGLE_TYPE>(SLALOM_SCAN_INVERT_CAMERA_ANGLE_PARAMS),
                    // Bidirectionnals
                    BT::BidirectionalPort<SLALOM_SCAN_POINTS_TYPE>(SLALOM_SCAN_POINTS_PARAMS)
                };
                default_ports.merge(new_ports);

                return default_ports;
            }

            BT::NodeStatus onStart() override;

        protected:
            SLALOM_SCAN_ACTUAL_ANGLE_TYPE _actual_angle;
            SLALOM_SCAN_INVERT_CAMERA_ANGLE_TYPE _invert_camera_angle;
            SlalomPointArray _points;
            size_t _frames_with_detection;

            void handle_success() override;
            void handle_failure() override;
            void flush_points();
            BT::NodeStatus get_detection_status() override;

            void ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) override;
    };
}
