//
// Created by albus on 2022/08/19.
//

#include "utils.h"

float drand48() {
    return float(((double) (rand()) / (RAND_MAX))); /* RAND_MAX = 32767 */
}

//RGB?l(0~255)??(0~1)????K??
Eigen::Vector3d rgbNormalize(const Eigen::Vector3d rgb) {
    Eigen::Vector3d out_rgb{rgb.x() / 255, rgb.y() / 255, rgb.z() / 255};
    return out_rgb;
}
