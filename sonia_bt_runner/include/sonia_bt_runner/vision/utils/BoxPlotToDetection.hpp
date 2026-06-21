#pragma once

#include <vector>
#include "sonia_common_ros2/msg/detection.hpp"

void
boxPlotToDetection (std::vector<sonia_common_ros2::msg::Detection>& detection_array);