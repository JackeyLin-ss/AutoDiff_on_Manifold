#pragma once

#include <pangolin/pangolin.h>
#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include "type.h"

inline Vec3d MakeJet3B(float id) {
  if (id <= 0) return Vec3d(128, 0, 0);
  if (id >= 1) return Vec3d(0, 0, 128);

  int icp = (id * 8);
  float ifP = (id * 8) - icp;

  if (icp == 0) return Vec3d(255 * (0.5 + 0.5 * ifP), 0, 0);
  if (icp == 1) return Vec3d(255, 255 * (0.5 * ifP), 0);
  if (icp == 2) return Vec3d(255, 255 * (0.5 + 0.5 * ifP), 0);
  if (icp == 3) return Vec3d(255 * (1 - 0.5 * ifP), 255, 255 * (0.5 * ifP));
  if (icp == 4)
    return Vec3d(255 * (0.5 - 0.5 * ifP), 255, 255 * (0.5 + 0.5 * ifP));
  if (icp == 5) return Vec3d(0, 255 * (1 - 0.5 * ifP), 255);
  if (icp == 6) return Vec3d(0, 255 * (0.5 - 0.5 * ifP), 255);
  if (icp == 7) return Vec3d(0, 0, 255 * (1 - 0.5 * ifP));
  return Vec3d(255, 255, 255);
}

class Viewer {
 public:
  Viewer();

  ~Viewer();

  void Run();

  void LoadMappoints(const std::string &files);

  void SetGroundTruthPose(const Mat44d &pose);

  void SetEstimatePose(const Mat44d &pose);

  void SetInitialPose(const Mat44d &pose);

 private:
  void DrawAxis();

  void DrawMapPoints();

  void DrawGroundTruthPose();

  void DrawEstimatePose();

  void DrawInitialPose();

  void DrawCameraWireframe(float r, float g, float b);

 private:
  std::mutex mutex_pose_;
  Mat44d ground_truth_pose_;
  Mat44d estimated_pose_;
  Mat44d initial_pose_;

  std::vector<Vec3d> points_;
  std::vector<Vec3d> colors_;
};
