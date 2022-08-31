//
// Created by albus on 2022/08/19.
//

#ifndef RENDERING2022_TRIANGLEOBJ_H
#define RENDERING2022_TRIANGLEOBJ_H

#include "Eigen/Dense"
#include "./Ray.h"
#include "./RayHitInfo.h"

class TriangleObj {
public:
    TriangleObj();
    TriangleObj(const Eigen::Vector3d & _v1,
                const Eigen::Vector3d & _v2,
                const Eigen::Vector3d & _v3,
                const Eigen::Vector3d & _color,
                const std::string & _color_name,
                const bool _is_light,
                const double _kd);

    bool rayIntersect(const Ray &in_Ray, RayHitInfo &out_Result);

    Eigen::Vector3d v1;
    Eigen::Vector3d v2;
    Eigen::Vector3d v3;
    Eigen::Vector3d normal;
    Eigen::Vector3d color;
    std::string color_name;
    double area;
    bool is_light;
    double kd;
};

#endif //RENDERING2022_TRIANGLEOBJ_H
