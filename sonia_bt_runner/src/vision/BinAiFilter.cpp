#include "sonia_bt_runner/vision/BinAiFilter.hpp"
#include "sonia_bt_runner/utils/NormalizeDetection.hpp"

#include <algorithm>
#include <tuple>

namespace vision{

    BinAiFilter::BinAiFilter(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : AbstractAiFilter(name, config, node)
    {}

    void BinAiFilter::handle_success()
    {
        RCLCPP_INFO(get_logger(), "onRunning success!!!");
        // We need to make some selection in the image array
        RCLCPP_INFO(get_logger(), "Getting the information because enough detection have been made : %ld detection(s)", _bin_array.size());
        
        setOutput(BIN_AI_FILTER_DETECTIONS, _bin_array);

        AbstractAiFilter::handle_success();
    }

    void BinAiFilter::ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg)
    {
        AbstractAiFilter::ai_filter_callback(msg);
        // BIN_AI_FILTER_DETECTIONS_TYPE bin;
	std::vector<std::tuple<AiDetection, Point>> all_bins;
        for (auto msg_obj : msg.detected_object) {
            if (msg_obj.class_name == "BOX_BIN") {
                utils::normalize_detection(msg_obj);
		AiDetection detection;
		detection.distance = msg_obj.distance;
		detection.confidence = msg_obj.confidence;
		detection.angle_teta = msg_obj.angle_teta;
		detection.angle_alpha = msg_obj.angle_alpha;
		detection.distance_teta = msg_obj.distance_teta;
		detection.distance_beta = msg_obj.distance_beta;
                Point point;
                point.x = msg_obj.distance * std::cos(msg_obj.angle_alpha * DEG_TO_RAD);
                point.y = msg_obj.distance * std::sin(-msg_obj.angle_alpha * DEG_TO_RAD);
		point.z = 0;
                all_bins.push_back(std::make_tuple(detection, point));
            }
        }

        // We verify if we have more bins that in the array or we have to much detection
        if (all_bins.size() <= _bin_array.size() || all_bins.size() > 4) {
            return;
        }

        // We sort the detection by y value
        std::sort(all_bins.begin(), all_bins.end(), [](const auto& a, const auto& b) {
	    return std::get<1>(a).y < std::get<1>(b).y;
        });

        // We compute the distance between the detection to verify if they are valid
        for (size_t i{};i < all_bins.size() - 1;i++) {
	    const auto& current_bin_point = std::get<1>(all_bins[i]);
	    const auto& next_bin_point = std::get<1>(all_bins[i + 1]);
            double ddistance{(next_bin_point.x - current_bin_point.x) * (next_bin_point.x - current_bin_point.x) + (next_bin_point.y - current_bin_point.y) * (next_bin_point.y - current_bin_point.y)};
            float fdistance= static_cast<float>(ddistance);

            if (std::sqrt(fdistance) > DISTANCE_BETWEEN_BIN) {
		   return;
            }
        }

	_bin_array.reserve(all_bins.size());
	std::transform(all_bins.begin(), all_bins.end(), _bin_array.begin(), [](const auto &a) {
		return std::get<0>(a);
	});
    }

    BT::NodeStatus BinAiFilter::get_detection_status()
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
