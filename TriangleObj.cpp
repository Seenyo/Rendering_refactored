//
// Created by albus on 2022/08/19.
//

#include "./TriangleObj.h"
#include "./utils.h"
#include "./constants.h"

TriangleObj::TriangleObj() = default;

TriangleObj::TriangleObj(const Eigen::Vector3d & _v1,
                         const Eigen::Vector3d & _v2,
                         const Eigen::Vector3d & _v3,
                         const Eigen::Vector3d & _color,
                         const std::string & _color_name,
                         const bool _is_light,
                         const double _kd):
                         v1(_v1),
                         v2(_v2),
                         v3(_v3),
                         normal(((v2 - v1).cross(v3 - v1)).normalized()),
                         color(rgbNormalize(_color)),
                         color_name(_color_name),
                         area((((v2 - v1).cross(v3 - v1)) / 2).norm()),
                         is_light(_is_light),
                         kd(_kd){}


bool TriangleObj::rayIntersect(const Ray &in_Ray, RayHitInfo &out_Result) {
    out_Result.t = __FAR__;

    double denominator = normal.dot(in_Ray.d);

    if (denominator >= 0.0)
        return false;

    const double t = normal.dot(v3 - in_Ray.o) / denominator;
    if (t <= 0.0)
        return false;

    const Eigen::Vector3d x = in_Ray.o + t * in_Ray.d;

    Eigen::Matrix<double, 3, 2> A;
    A.col(0) = v1 - v3;
    A.col(1) = v2 - v3;

    Eigen::Matrix2d ATA = A.transpose() * A;
    const Eigen::Vector2d b = A.transpose() * (x - v3);

    const Eigen::Vector2d alpha_beta = ATA.inverse() * b;

    //?O?p?`?????????
    if (alpha_beta.x() < 0.0 || 1.0 < alpha_beta.x() || alpha_beta.y() < 0.0 ||
        1.0 < alpha_beta.y() || (1 - alpha_beta.x() - alpha_beta.y()) < 0.0 ||
        1.0 < (1 - alpha_beta.x() - alpha_beta.y()))
        return false;

    out_Result.t = t;
    out_Result.alpha = alpha_beta.x();
    out_Result.beta = alpha_beta.y();
    return true;
}