#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

using Mat44d = Eigen::Matrix4d;
using Mat33d = Eigen::Matrix3d;
using Vec6d = Eigen::Matrix<double, 6, 1, Eigen::ColMajor>;
using Vec7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
using Vec3d = Eigen::Vector3d;
using Vec2d = Eigen::Vector2d;

struct CameraInt {
  double fx;
  double fy;
  double cx;
  double cy;
  CameraInt() : fx(0), fy(0), cx(0), cy(0) {}
  CameraInt(double _fx, double _fy, double _cx, double _cy)
      : fx(_fx), fy(_fy), cx(_cx), cy(_cy) {}
};
