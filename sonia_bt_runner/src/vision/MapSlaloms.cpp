#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

#include "sonia_bt_runner/vision/MapSlaloms.hpp"

namespace vision {
MapSlaloms::MapSlaloms(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
	: AbstractAiFilter(name, config, node)
{
}

BT::NodeStatus MapSlaloms::onStart()
{
	auto status = AbstractAiFilter::onStart();

	switch (status) {
		case BT::NodeStatus::RUNNING:
			_side = getInput<MAP_SLALOMS_SIDE_TYPE>(MAP_SLALOMS_SIDE).value();
			_slalom_side = getInput<MAP_SLALOMS_SLALOM_SIDE_TYPE>(MAP_SLALOMS_SLALOM_SIDE).value();
			_step_distance = getInput<MAP_SLALOMS_STEP_DISTANCE_TYPE>(MAP_SLALOMS_STEP_DISTANCE).value();

			handle_status(status);

			break;
	    default:
			break;
	}

	return status;
}

double MapSlaloms::calculate_detection_distance_average() const noexcept
{
	if (_detection_array.empty()) {
		return 0.0;
	}

	double distance_average = 0.0;

	for (const auto &detection : _detection_array) {
		distance_average += detection.distance;
	}

	distance_average /= _detection_array.size();

	return distance_average;
}

double MapSlaloms::calculate_detection_distance_standard_deviation() const noexcept
{
	if (_detection_array.empty()) {
		return 0.0;
	}

	double average = calculate_detection_distance_average();
	double diff = 0.0;

	for (const auto &detection : _detection_array) {
		diff += std::pow(detection.distance - average, 2);
	}

	diff /= _detection_array.size();

	return std::sqrt(diff);
}

void MapSlaloms::classify_detections(sonia_common_ros2::msg::Detection out_detections[SLALOM_LAYER][SLALOM_PER_LAYER]) const noexcept
{
	// - = white slalom
	// * = red slalom
	//
	//          -          *          -      <- layer 0 (nearest line)
	//
	//          -          *          -      <- layer 1
	//
	//          -          *          -      <- layer 2
	//
	// We split the detections into lines (layers) by depth, then order every
	// line left -> right by its horizontal bearing (angle_alpha).

	if (_detection_array.empty()) {
		return;
	}

	std::vector<sonia_common_ros2::msg::Detection> detections = _detection_array;
	std::sort(detections.begin(), detections.end(),
		[](const sonia_common_ros2::msg::Detection &a, const sonia_common_ros2::msg::Detection &b) {
			return a.distance < b.distance;
		});

	// A depth gap larger than this means the next detection belongs to a new line.
	double layer_gap = std::max(calculate_detection_distance_standard_deviation(),
								static_cast<double>(MAP_SLALOMS_MIN_LAYER_GAP_M));

	std::vector<std::vector<sonia_common_ros2::msg::Detection>> layers;
	layers.emplace_back();
	layers.back().push_back(detections.front());

	for (size_t i = 1; i < detections.size(); ++i) {
		bool new_layer = detections[i].distance - detections[i - 1].distance > layer_gap;

		if (new_layer && layers.size() < SLALOM_LAYER) {
			layers.emplace_back();
		}

		layers.back().push_back(detections[i]);
	}

	for (size_t l = 0; l < layers.size() && l < SLALOM_LAYER; ++l) {
		auto &layer = layers[l];

		std::sort(layer.begin(), layer.end(),
			[](const sonia_common_ros2::msg::Detection &a, const sonia_common_ros2::msg::Detection &b) {
				return a.angle_alpha < b.angle_alpha;
			});

		for (size_t s = 0; s < layer.size() && s < SLALOM_PER_LAYER; ++s) {
			out_detections[l][s] = layer[s];
		}
	}
}

int MapSlaloms::estimate_position(const sonia_common_ros2::msg::Detection front_layer[SLALOM_PER_LAYER]) const noexcept
{
	// We only reason about the nearest line: a red seen on a deeper line does
	// not count as "facing the red". An empty slot has a default confidence of 0.
	const sonia_common_ros2::msg::Detection *red = nullptr;
	std::vector<const sonia_common_ros2::msg::Detection *> whites;

	for (int s = 0; s < SLALOM_PER_LAYER; ++s) {
		const auto &detection = front_layer[s];

		if (detection.confidence <= 0.0f) {
			continue;
		}

		if (detection.class_name.compare("RED_SLALOM") == 0) {
			red = &detection;
		} else if (detection.class_name.compare("WHITE_SLALOM") == 0) {
			whites.push_back(&detection);
		}
	}

	if (whites.empty() && red == nullptr) {
		return -1;
	}

	if (red != nullptr && !whites.empty()) {
		// A red and a white share the nearest line.
		if (whites.size() >= 2) {
			// The red sits between two whites -> we face the red (center).
			return 3;
		}

		// One white: angle_alpha > 0 means the object is on the right. If the red
		// is to the right of the white we face the left white (position 2),
		// otherwise we face the right white (position 4).
		return red->angle_alpha > whites[0]->angle_alpha ? 2 : 4;
	}

	// Only white(s) on the nearest line -> we are at an end. Slalom_side tells us
	// which end: the slalom is on the sub's left -> we are at the right end.
	return _slalom_side.compare("Right") == 0 ? 1 : 5;
}

bool MapSlaloms::compute_row_aligned_step(const sonia_common_ros2::msg::Detection front_layer[SLALOM_PER_LAYER], bool move_right, TrajectoryPose &out_pose) const noexcept
{
	// We need two poles on the front line to know how the slalom row is oriented.
	// The front line is already sorted left -> right by angle_alpha, so the first
	// and last non-empty slots are the outermost visible poles.
	const sonia_common_ros2::msg::Detection *left = nullptr;
	const sonia_common_ros2::msg::Detection *right = nullptr;

	for (int s = 0; s < SLALOM_PER_LAYER; ++s) {
		if (front_layer[s].confidence <= 0.0f) {
			continue;
		}

		if (left == nullptr) {
			left = &front_layer[s];
		}

		right = &front_layer[s];
	}

	if (left == nullptr || right == nullptr || left == right) {
		return false;
	}

	constexpr double DEG_TO_RAD = M_PI / 180.0;

	// Pole positions in the body frame (x forward, y left; angle_alpha > 0 = right).
	double lx = left->distance * std::cos(left->angle_alpha * DEG_TO_RAD);
	double ly = -left->distance * std::sin(left->angle_alpha * DEG_TO_RAD);
	double rx = right->distance * std::cos(right->angle_alpha * DEG_TO_RAD);
	double ry = -right->distance * std::sin(right->angle_alpha * DEG_TO_RAD);

	// Direction of the row, pointing from the left pole to the right pole.
	double dx = rx - lx;
	double dy = ry - ly;
	double norm = std::hypot(dx, dy);

	if (norm <= 0.0) {
		return false;
	}

	dx /= norm;
	dy /= norm;

	// Strafe one step along the row towards the target (rightward for higher
	// position numbers, which is the left -> right direction).
	double sign = move_right ? 1.0 : -1.0;

	out_pose = TrajectoryPose{};
	out_pose.positionX = static_cast<float>(sign * _step_distance * dx);
	out_pose.positionY = static_cast<float>(sign * _step_distance * dy);
	out_pose.frame = 1;

	return true;
}

void MapSlaloms::handle_success()
{
	sonia_common_ros2::msg::Detection grid[SLALOM_LAYER][SLALOM_PER_LAYER];
	classify_detections(grid);

	int position = estimate_position(grid[0]);
	setOutput<MAP_SLALOMS_CURRENT_POSITION_TYPE>(MAP_SLALOMS_CURRENT_POSITION, position);

	// Side tells us the side the sub should take, which maps to the target position.
	int target = _side.compare("Left") == 0 ? 2 : 4;

	Trajectory trajectory = getInput<MAP_SLALOMS_TRAJECTORY_TYPE>(MAP_SLALOMS_TRAJECTORY).value_or(Trajectory{});

	if (position != -1 && position != target) {
		bool move_right = target > position;

		TrajectoryPose row_step{};
		if (compute_row_aligned_step(grid[0], move_right, row_step)) {
			// Two front-line poles visible (positions 2/3/4): we know the row's
			// orientation, so we slide one step straight along it. The bearings
			// are measured from the sub's current heading, so this is correct for
			// any orientation without a separate rotation.
			trajectory.trajectory.push_back(row_step);
		} else {
			// Only one front-line pole (positions 1/5): the row orientation is not
			// observable. We fall back to facing the front line, then strafing in
			// the body frame from that known heading.
			float front_bearing = 0.0f;
			int front_count = 0;

			for (int s = 0; s < SLALOM_PER_LAYER; ++s) {
				if (grid[0][s].confidence > 0.0f) {
					front_bearing += grid[0][s].angle_alpha;
					++front_count;
				}
			}

			if (front_count > 0) {
				front_bearing /= front_count;
			}

			// Pose 1: face the front line (angle_alpha and orientationZ are in degrees).
			TrajectoryPose aim{};
			aim.orientationZ = -front_bearing;
			aim.frame = 1;
			trajectory.trajectory.push_back(aim);

			// Pose 2: strafe one position towards the target. Higher position
			// numbers are to the right, which is -Y in the body frame.
			TrajectoryPose strafe{};
			strafe.positionY = move_right ? -_step_distance : _step_distance;
			strafe.frame = 1;
			trajectory.trajectory.push_back(strafe);
		}
	}

	setOutput<MAP_SLALOMS_TRAJECTORY_TYPE>(MAP_SLALOMS_TRAJECTORY, trajectory);

	RCLCPP_INFO(get_logger(), "MapSlaloms: estimated position = %d, target = %d", position, target);

	AbstractAiFilter::handle_success();
}

BT::NodeStatus MapSlaloms::get_detection_status()
{
	switch (AbstractAiFilter::get_detection_status()) {
		case BT::NodeStatus::FAILURE:
			return BT::NodeStatus::FAILURE;
		case BT::NodeStatus::RUNNING:
			if (_detection_array.size() < detection_number_for_average()) {
				RCLCPP_INFO(get_logger(), "MapSlaloms: not enough detections yet (%ld)", _detection_array.size());

				return BT::NodeStatus::RUNNING;
			}

			return BT::NodeStatus::SUCCESS;
		default:
			assert(0 && "Not expected status");
	}

	return BT::NodeStatus::FAILURE;
}

void MapSlaloms::ai_filter_callback(const sonia_common_ros2::msg::DetectionArray &msg)
{
	AbstractAiFilter::ai_filter_callback(msg);

	for (const auto &msg_obj : msg.detected_object) {
		bool is_white = msg_obj.class_name.compare("WHITE_SLALOM") == 0;
		bool is_red = msg_obj.class_name.compare("RED_SLALOM") == 0;

		if ((is_white || is_red) &&
			msg_obj.confidence >= confidence() &&
			msg_obj.distance <= max_depth()) {
			_detection_array.push_back(msg_obj);
		}
	}
}
}
