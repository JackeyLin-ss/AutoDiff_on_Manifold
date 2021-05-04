#pragma once

#include "colordef.h"
#include "se3.h"
#include "type.h"

#include <fstream>

class PoseEstimationProblem {
 public:
  PoseEstimationProblem() {
    num_parameters_ = 6;  // rotation  3dof  translation 3dof
  }
  ~PoseEstimationProblem() { delete[] parameters_; }

  int num_observations() const { return num_observations_; }

  Vec2d observation(int i) const { return observations_[i]; }

  double *mutable_camera_pose() { return parameters_; }

  Vec3d point_position_for_observation(int i) const { return points_[i]; }

  bool setInitialParameter(const double *const data) {
    memcpy(parameters_, data,
           sizeof(double) * static_cast<unsigned long>(num_parameters_));
    return true;
  }

  bool loadFile(const std::string &filename) {
    std::ifstream fin;
    fin.open(filename);
    if (!fin.is_open()) {
      std::cout << kColorRed << " open file " << filename << " not found "
                << kColorReset << std::endl;
      return false;
    }

    std::string line;
    std::getline(fin, line);
    sscanf(line.c_str(), " %d ", &num_observations_);

    num_points_ = num_observations_;
    num_parameters_ = 6;
    points_.resize(num_points_);
    parameters_ = new double[num_parameters_];

    observations_.resize(num_observations_);

    for (int i = 0; i < num_observations(); ++i) {
      // 3d point
      std::getline(fin, line);
      double x, y, z;
      sscanf(line.c_str(), "%lf %lf %lf", &x, &y, &z);
      points_[i] = Vec3d(x, y, z);

      // 2d feature points;
      std::getline(fin, line);
      double u, v;
      sscanf(line.c_str(), "%lf %lf", &u, &v);
      observations_[i] = Vec2d(u, v);
    }

    fin.close();
    return true;
  }

 private:
  // points' position is fixed
  int num_points_;
  int num_observations_;
  int num_parameters_;

  std::vector<Vec2d> observations_;
  std::vector<Vec3d> points_;
  double *parameters_;
};
