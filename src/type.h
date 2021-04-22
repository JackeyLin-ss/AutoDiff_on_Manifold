#pragma once

#include <Eigen/Dense>

typedef Eigen::Vector3f Vec3_t;
typedef Eigen::Matrix4d Mat44t;

struct CameraInt {
  float fx;
  float fy;
  float cx;
  float cy;
  CameraInt() : fx(0), fy(0), cx(0), cy(0) {}
  CameraInt(float _fx, float _fy, float _cx, float _cy)
      : fx(_fx), fy(_fy), cx(_cx), cy(_cy) {}
};
