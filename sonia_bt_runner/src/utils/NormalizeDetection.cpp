#include <cstdlib>
#include <cstring>
#include <cassert>

#include "sonia_bt_runner/utils/NormalizeDetection.hpp"
#include "sonia_bt_runner/utils/AUV.hpp"

namespace utils {
#define AUV8_WATER_REFRACTION 1.33
// TODO: Check water refraction for LITE1
#define LITE1_WATER_REFRACTION 1.33

void normalize_detection(sonia_common_ros2::msg::Detection &detection, float distance_offset) {
	switch (get_auv()) {
		case SUB_AUV8:
			detection.distance *= AUV8_WATER_REFRACTION;
			detection.distance += distance_offset;
			detection.distance_teta *= AUV8_WATER_REFRACTION;
			detection.distance_beta *= AUV8_WATER_REFRACTION;

			break;
		case SUB_LITE1:
			detection.distance *= LITE1_WATER_REFRACTION;
			detection.distance += distance_offset;
			detection.distance_teta *= LITE1_WATER_REFRACTION;
			detection.distance_beta *= LITE1_WATER_REFRACTION;

			break;
		default:
			assert(0 && "Unknown AUV");
	}
}
}
