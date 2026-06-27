#pragma once

#include <vector>

#include "sonia_bt_runner/vision/AbstractAiFilter.hpp"
#include "sonia_bt_runner/utils/Point.hpp"
#include "sonia_bt_runner/utils/Trajectory.hpp"

#define MAP_SLALOMS_SIDE "Side"
#define MAP_SLALOMS_SIDE_PARAMS MAP_SLALOMS_SIDE, "The side, the sub should take for the slalom (Left or Right)"
#define MAP_SLALOMS_SIDE_TYPE std::string

#define MAP_SLALOMS_SLALOM_SIDE "Slalom_side"
#define MAP_SLALOMS_SLALOM_SIDE_PARAMS MAP_SLALOMS_SLALOM_SIDE, "The side where the slalom is from the sub position (Left or Right)"
#define MAP_SLALOMS_SLALOM_SIDE_TYPE std::string

#define MAP_SLALOMS_CURRENT_POSITION "Position"
#define MAP_SLALOMS_CURRENT_POSITION_PARAMS MAP_SLALOMS_CURRENT_POSITION, -1, "The current position of the sub around the slalom (1, 2, 3, 4, 5 or -1)"
#define MAP_SLALOMS_CURRENT_POSITION_TYPE int

#define MAP_SLALOMS_STEP_DISTANCE "Step_distance"
#define MAP_SLALOMS_STEP_DISTANCE_PARAMS MAP_SLALOMS_STEP_DISTANCE, 0.5f, "Lateral distance in meters travelled for one position step"
#define MAP_SLALOMS_STEP_DISTANCE_TYPE float

#define MAP_SLALOMS_TRAJECTORY "Trajectory"
#define MAP_SLALOMS_TRAJECTORY_PARAMS MAP_SLALOMS_TRAJECTORY, "The trajectory the sub should follow to reach the target position"
#define MAP_SLALOMS_TRAJECTORY_TYPE Trajectory

#define SLALOM_LAYER 3
#define SLALOM_PER_LAYER 3

// Two detections whose depth differ by more than this gap are considered to
// belong to two different slalom layers (lines).
#define MAP_SLALOMS_MIN_LAYER_GAP_M 0.5f

namespace vision {
class MapSlaloms : public AbstractAiFilter {
public:
	MapSlaloms(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);

	static BT::PortsList providedPorts()
	{
		auto default_ports = AbstractAiFilter::providedPorts();
		BT::PortsList new_ports = {
			// Inputs
			BT::InputPort<MAP_SLALOMS_SIDE_TYPE>(MAP_SLALOMS_SIDE_PARAMS),
			BT::InputPort<MAP_SLALOMS_SLALOM_SIDE_TYPE>(MAP_SLALOMS_SLALOM_SIDE_PARAMS),
			BT::InputPort<MAP_SLALOMS_STEP_DISTANCE_TYPE>(MAP_SLALOMS_STEP_DISTANCE_PARAMS),
			BT::BidirectionalPort<MAP_SLALOMS_CURRENT_POSITION_TYPE>(MAP_SLALOMS_CURRENT_POSITION_PARAMS),
			// Outputs
			BT::BidirectionalPort<MAP_SLALOMS_TRAJECTORY_TYPE>(MAP_SLALOMS_TRAJECTORY_PARAMS)
		};

		default_ports.merge(new_ports);

		return default_ports;
	}

	BT::NodeStatus onStart() override;
protected:
	void handle_success() override;
	BT::NodeStatus get_detection_status() override;
	void ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg) override;
private:
	double calculate_detection_distance_average() const noexcept;
	double calculate_detection_distance_standard_deviation() const noexcept;
	void classify_detections(sonia_common_ros2::msg::Detection out_detections[SLALOM_LAYER][SLALOM_PER_LAYER]) const noexcept;
	int estimate_position(const sonia_common_ros2::msg::Detection front_layer[SLALOM_PER_LAYER]) const noexcept;
	bool compute_row_aligned_step(const sonia_common_ros2::msg::Detection front_layer[SLALOM_PER_LAYER], bool move_right, TrajectoryPose &out_pose) const noexcept;

	MAP_SLALOMS_SIDE_TYPE _side;
	MAP_SLALOMS_SLALOM_SIDE_TYPE _slalom_side;
	MAP_SLALOMS_STEP_DISTANCE_TYPE _step_distance = 0.5f;
	std::vector<sonia_common_ros2::msg::Detection> _detection_array;
};
}
