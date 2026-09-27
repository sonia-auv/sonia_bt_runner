#include "sonia_bt_runner/vision/utils/BoxPlotToDetection.hpp"

namespace vision {

    void
    boxPlotToDetection (std::vector<sonia_common_ros2::msg::Detection>& detection_array)
    {
        float q1{};             // first quartile
        float q3{};             // third quartile
        float min{};            // min bound
        float max{};            // max bound

        // We made a filtering in ascending order of the teta distance
        sonia_common_ros2::msg::Detection temp;
        for (size_t i{}; i < detection_array.size() - 1; ++i) {
            for (size_t j{}; j < detection_array.size() - i - 1; ++j) {
                if (detection_array[j].distance_teta > detection_array[j + 1].distance_teta) {
                    temp = detection_array[j];
                    detection_array[j] = detection_array[j + 1];
                    detection_array[j + 1] = temp;
                }
            }
        }

        // We compute the quartile
        if (detection_array.size() % 4) {
            q1 = detection_array[detection_array.size()/4.0f].distance_teta;
            q3 = detection_array[detection_array.size()*3.0f/4.0f].distance_teta;
        } else {
            q1 = (detection_array[(size_t)(detection_array.size()/4)].distance_teta + detection_array[(size_t)(detection_array.size()/4) + (size_t)1].distance_teta)/2.0f;
            q3 = (detection_array[(size_t)(detection_array.size()*3/4)].distance_teta + detection_array[(size_t)(detection_array.size()*3/4) + (size_t)1].distance_teta)/2.0f;
        }

        // We compute the min and max bound where the data will be kept
        min = q1 - 1.5f * (q3 - q1);
        max = q3 + 1.5f * (q3 - q1);

        // We keep the detection inside the min and the max bound
        if (detection_array.size() > 2) {
            for (std::vector<sonia_common_ros2::msg::Detection>::iterator it = detection_array.begin(); it != detection_array.end();) {
                if (it->distance_teta < min || it->distance_teta > max) {
                it = detection_array.erase(it);
                } else {
                ++it;
                }
            }
        }

        // We did the same thing with the distance_beta
        // We made a filtering in ascending order of the beta distance
        for (size_t i{}; i < detection_array.size() - 1; ++i) {
            for (size_t j{}; j < detection_array.size() - i - 1; ++j) {
                if (detection_array[j].distance_beta > detection_array[j + 1].distance_beta) {
                    temp = detection_array[j];
                    detection_array[j] = detection_array[j + 1];
                    detection_array[j + 1] = temp;
                }
            }
        }

        // We compute the quartile
        if (detection_array.size() % 4) {
            q1 = detection_array[detection_array.size()/4.0f].distance_beta;
            q3 = detection_array[detection_array.size()*3.0f/4.0f].distance_beta;
        } else {
            q1 = (detection_array[(size_t)(detection_array.size()/4)].distance_beta + detection_array[(size_t)(detection_array.size()/4) + 1].distance_beta)/2.0f;
            q3 = (detection_array[(size_t)(detection_array.size()*3/4)].distance_beta + detection_array[(size_t)(detection_array.size()*3/4) + 1].distance_beta)/2.0f;
        }

        // We compute the min and max bound where the data will be kept
        min = q1 - 1.5f * (q3 - q1);
        max = q3 + 1.5f * (q3 - q1);

        // We keep the detection inside the min and the max bound
        if (detection_array.size() > 2) {
            for (std::vector<sonia_common_ros2::msg::Detection>::iterator it = detection_array.begin(); it != detection_array.end();) {
                if (it->distance_beta < min || it->distance_beta > max) {
                    it = detection_array.erase(it);
                } else {
                    ++it;
                }
            }
        }
        // We did the same thing with the depth
        // We made a filtering in ascending order of the depth
        for (size_t i{}; i < detection_array.size() - 1; ++i) {
            for (size_t j{}; j < detection_array.size() - i - 1; ++j) {
                if (detection_array[j].distance > detection_array[j + 1].distance) {
                    temp = detection_array[j];
                    detection_array[j] = detection_array[j + 1];
                    detection_array[j + 1] = temp;
                }
            }
        }

        // We compute the quartile
        if (detection_array.size() % 4) {
            q1 = detection_array[detection_array.size()/4.0f].distance;
            q3 = detection_array[detection_array.size()*3.0f/4.0f].distance;
        } else {
            q1 = (detection_array[(size_t)(detection_array.size()/4)].distance + detection_array[(size_t)(detection_array.size()/4) + 1].distance)/2.0f;
            q3 = (detection_array[(size_t)(detection_array.size()*3/4)].distance + detection_array[(size_t)(detection_array.size()*3/4) + 1].distance)/2.0f;
        }

        // We compute the min and max bound where the data will be kept
        min = q1 - 1.5f * (q3 - q1);
        max = q3 + 1.5f * (q3 - q1);

        // We keep the detection inside the min and the max bound
        if (detection_array.size() > 2) {
            for (std::vector<sonia_common_ros2::msg::Detection>::iterator it = detection_array.begin(); it != detection_array.end();) {
                if (it->distance < min || it->distance > max) {
                    it = detection_array.erase(it);
                } else {
                    ++it;
                }
            }
        }
        
            // Will propably do the same thing with the depth
    }

}