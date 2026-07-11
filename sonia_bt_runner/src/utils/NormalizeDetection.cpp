#include <cstdlib>
#include <cstring>
#include <cassert>

#include "sonia_bt_runner/utils/NormalizeDetection.hpp"

namespace utils {
#define AUV8_WATER_REFRACTION 1.33
// TODO: Check water refraction for LITE1
#define LITE1_WATER_REFRACTION 1.33

#define SUB_AUV8 0
#define SUB_LITE1 1
#define SUB_NONE -1
static int sub = SUB_NONE;

void normalize_detection(sonia_common_ros2::msg::Detection &detection) {
	if (sub == SUB_NONE) {
		char *auv = getenv("AUV");

		if (auv) {
			if (!strcmp(auv, "AUV8")) {
				sub =  SUB_AUV8;
			} else if (!strcmp(auv, "LITE1")) {
				sub = SUB_LITE1;
			} else {
				assert(0 && "Unknow AUV");
			}
		} else {
			assert(0 && "Unknow AUV");
		}
	}

	switch (sub) {
		case SUB_AUV8:
			detection.distance *= AUV8_WATER_REFRACTION;
			detection.distance_teta *= AUV8_WATER_REFRACTION;
			detection.distance_beta *= AUV8_WATER_REFRACTION;

			break;
		case SUB_LITE1:
			detection.distance *= LITE1_WATER_REFRACTION;
			detection.distance_teta *= LITE1_WATER_REFRACTION;
			detection.distance_beta *= LITE1_WATER_REFRACTION;

			break;
		default:
			assert(0 && "Unknown AUV");
	}
}
}
