#include "sonia_bt_runner/vision/AbstractAiFilter.hpp"
#include "sonia_bt_runner/utils/Point.hpp"

#define DISTANCE_BETWEEN_BIN 3

#define RAD_TO_DEG (180.0f/M_PI)
#define DEG_TO_RAD (M_PI/180.0f)

#define BIN_AI_FILTER_DETECTIONS "Detections"
#define BIN_AI_FILTER_DETECTIONS_PARAMS BIN_AI_FILTER_DETECTIONS, "The detected bin"
#define BIN_AI_FILTER_DETECTIONS_TYPE std::vector<Point>

namespace vision {

    class BinAiFilter : public AbstractAiFilter{
        public:
            BinAiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
            ~BinAiFilter() override = default;
            static BT::PortsList providedPorts()
            {
                auto default_ports = AbstractAiFilter::providedPorts();
                BT::PortsList new_ports = {
                        // Outputs
                        BT::OutputPort<BIN_AI_FILTER_DETECTIONS_TYPE>(BIN_AI_FILTER_DETECTIONS_PARAMS)
                };
                default_ports.merge(new_ports);
                return default_ports;
            }

            void ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) override;
            BT::NodeStatus get_detection_status() override;
            void handle_success() override;

        private:
            BIN_AI_FILTER_DETECTIONS_TYPE _bin_array;
    };

}