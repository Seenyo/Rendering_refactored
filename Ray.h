//
// Created by albus on 2022/08/19.
//

#ifndef RENDERING2022_RAY_H
#define RENDERING2022_RAY_H

#include <Eigen/Dense>

struct Ray
{
    Eigen::Vector3d o;
    Eigen::Vector3d d;
};

#endif //RENDERING2022_RAY_H
