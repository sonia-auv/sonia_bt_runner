#include "sonia_bt_runner/utils/CONSTANT.hpp"

#include "sonia_bt_runner/utils/AUV.hpp"

namespace utils
{
    float get_camera_to_middle() { return get_auv() == SUB_AUV8 ? AUV8_CAMERA_TO_MIDDLE : LITE1_CAMERA_TO_MIDDLE; }
}  // namespace utils
