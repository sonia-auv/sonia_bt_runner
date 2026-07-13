#include "sonia_bt_runner/vision/BinAiFilter.hpp"
#include "sonia_bt_runner/utils/NormalizeDetection.hpp"

namespace utils{

    BinAiFilter::BinAiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : AbstractAiFilter(name, config, node)
    {}

    void BinAiFilter::handle_success()
    {
        RCLCPP_INFO(get_logger(), "onRunning success!!!");
        // We need to make some selection in the image array
        RCLCPP_INFO(get_logger(), "Getting the information because enough detection have been made : %ld detection(s)", _detection_array.size());
        
        setOutput(BIN_AI_FILTER_DETECTIONS, _bin_array);

        AbstractAiFilter::handle_success();
    }

    void BinAiFilter::ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg)
    {
        AbstractAiFilter::ai_filter_callback(msg);
        BIN_AI_FILTER_DETECTIONS_TYPE bin;
        for (auto msg_obj : msg.detected_object) {
            if (msg_obj.object_class == "BOX_BIN") {
                utils::normalize_detection(msg_obj);
                Point point;
                point.x = msg_obj.distance * std::cos(msg_obj.angle_alpha * DEG_TO_RAD);
                point.y = msg_obj.distance * std::sin(-msg_obj.angle_alpha * DEG_TO_RAD);
                detection.push_back(point);
            }
        }

        // We verify if we have more bins that in the array or we have to much detection
        if (bin.size() <= _bin_array.size() || bin.size() > 4) {
            return;
        }

        // We sort the detection by y value
        std::sort(bin.begin(), bin.end(), [](const Point& a, const Point& b) {
            return a.y < b.y;
        });

        // We compute the distance between the detection to verify if they are valid
        for (size_t i{};i < bin.size() - 1;i++) {
            float distance{(bin[i + 1].x - bin[i].x) * (bin[i + 1].x - bin[i].x) + (bin[i + 1].y - bin[i].y) * (bin[i + 1].y - bin[i].y)};
            if (std::sqrt(distance) > DISTANCE_BETWEEN_BIN) {
                return;
            }
        }
        
        _bin_array = bin;
    }

    BT::NodeStatus get_detection_status()
    {
        switch (AbstractAiFilter::get_detection_status()) {
		    case BT::NodeStatus::FAILURE:
			    return BT::NodeStatus::FAILURE;
		    case BT::NodeStatus::RUNNING:
			    if (_bin_array.size() < 4) {
				    RCLCPP_INFO(get_logger(), "We don't have yet the number of detection we want");
				    return BT::NodeStatus::RUNNING;
			    }

			    return BT::NodeStatus::SUCCESS;
		    default:
			    assert(0 && "Not expected status");
	    }
    }
}