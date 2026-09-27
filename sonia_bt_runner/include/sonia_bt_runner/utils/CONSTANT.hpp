#pragma once

// Constant list for droppers and torpidoes3
const float CAMERA_TO_DROPPER_OFFSET_X = 0.0;
const float CAMERA_TO_DROPPER_OFFSET_Y = 0.0;

const float CAMERA_TO_TORPIDO_PEPPER_OFFSET_X = 0.07;
const float CAMERA_TO_TORPIDO_SALT_OFFSET_X = -0.17;
const float CAMERA_TO_TORPEDO_OFFSET_Y = 0.17;

enum class Side {left, right};
enum class Path {ServeyRepair, SearchRescue};

const float AUV8_CAMERA_TO_MIDDLE = 0.4;
const float LITE1_CAMERA_TO_MIDDLE = 0.2;

namespace utils {
float get_camera_to_middle();
}
