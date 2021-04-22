#pragma once

#include "type.h"
#include <Eigen/Dense>
#include <mutex>
#include <pangolin/pangolin.h>
#include <vector>

inline Vec3_t MakeJet3B(float id) {
  if (id <= 0)
    return Vec3_t(128, 0, 0);
  if (id >= 1)
    return Vec3_t(0, 0, 128);

  int icp = (id * 8);
  float ifP = (id * 8) - icp;

  if (icp == 0)
    return Vec3_t(255 * (0.5 + 0.5 * ifP), 0, 0);
  if (icp == 1)
    return Vec3_t(255, 255 * (0.5 * ifP), 0);
  if (icp == 2)
    return Vec3_t(255, 255 * (0.5 + 0.5 * ifP), 0);
  if (icp == 3)
    return Vec3_t(255 * (1 - 0.5 * ifP), 255, 255 * (0.5 * ifP));
  if (icp == 4)
    return Vec3_t(255 * (0.5 - 0.5 * ifP), 255, 255 * (0.5 + 0.5 * ifP));
  if (icp == 5)
    return Vec3_t(0, 255 * (1 - 0.5 * ifP), 255);
  if (icp == 6)
    return Vec3_t(0, 255 * (0.5 - 0.5 * ifP), 255);
  if (icp == 7)
    return Vec3_t(0, 0, 255 * (1 - 0.5 * ifP));
  return Vec3_t(255, 255, 255);
}

class Viewer {
public:
  Viewer();

  ~Viewer();

  void Run();

  void LoadMappoints(const std::string &files);

  void SetGroundTruthPose(const Mat44t &pose);

  void SetEstimatePose(const Mat44t &pose);

private:
  void DrawAxis();

  void DrawMapPoints();

  void DrawGroundTruthPose();

  void DrawEstimatePose();

  void DrawCameraWireframe(float r, float g, float b);

private:
  std::mutex mutex_pose_;
  Mat44t ground_truth_pose_;
  Mat44t estimated_pose_;

  std::vector<Vec3_t> points_;
  std::vector<Vec3_t> colors_;
};
