#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include "se3.h"
#include "type.h"

/**
 * @brief The PoseSE3Parameterization class for local parameterization (usauly
 * for analytic derivatives)
 */

class PoseSE3Parameterization : public ceres::LocalParameterization {
 public:
  PoseSE3Parameterization() {}
  virtual ~PoseSE3Parameterization() {}
  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const {
    Eigen::Map<const Eigen::Vector3d> trans(x + 3);
    SE3 se3_delta = SE3::exp(Eigen::Map<const Vec6d>(delta));
    Eigen::Quaterniond quaterd_plus =
        se3_delta.rotation() * toQuaterniond(Eigen::Map<const Vec3d>(x));
    Eigen::Map<Vec3d> angles_plus(x_plus_delta);
    angles_plus = toAngleAxis(quaterd_plus);

    Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 3);
    trans_plus = se3_delta.rotation() * trans + se3_delta.translation();
    return true;
  }
  virtual bool ComputeJacobian(const double *x, double *jacobian) const {
    // local size = global size = 6;
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::ColMajor> > J(jacobian);
    J.setIdentity();
    return true;
  }
  virtual int GlobalSize() const { return 6; }
  virtual int LocalSize() const { return 6; }
};

/**
 * @brief Reprojectionerror cost function ( analytic derivatives )
 */
class ReprojectionErrorSE3 : public ceres::SizedCostFunction<2, 6> {
 public:
  ReprojectionErrorSE3(const CameraInt &cam_int, double observation_x,
                       double observation_y, double px, double py, double pz)
      : cam_int_(cam_int),
        observation_x_(observation_x),
        observation_y_(observation_y),
        px_(px),
        py_(py),
        pz_(pz) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    // convert so3 vector to rotation matrix
    Eigen::Map<const Vec3d> rotation(parameters[0]);
    double angle = rotation.norm();
    Eigen::Vector3d axis = rotation.normalized();
    Eigen::AngleAxisd agas(angle, axis);
    auto rotation_mat = agas.toRotationMatrix();

    // translation vector
    Eigen::Map<const Eigen::Vector3d> trans(parameters[0] + 3);

    // step 1 compute residual

    Eigen::Vector3d point(px_, py_, pz_);
    Eigen::Vector3d p = rotation_mat * point + trans;

    double fx = cam_int_.fx;
    double fy = cam_int_.fy;
    double cx = cam_int_.cx;
    double cy = cam_int_.cy;

    residuals[0] = fx * p.x() / p.z() + cx - observation_x_;
    residuals[1] = fx * p.y() / p.z() + cy - observation_y_;

    // step 2 compute jacobian

    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_cam;

    J_cam << fx / p.z(), 0, -fx / (p.z() * p.z()) * p.x(), 0, fy / p.z(),
        -fy / (p.z() * p.z()) * p.y();
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> > J_se3(
            jacobians[0]);
        J_se3.block<2, 3>(0, 0) = -J_cam * skew(p);
        J_se3.block<2, 3>(0, 3) = J_cam;
      }
    }

    return true;
  }

 private:
  CameraInt cam_int_;
  double observation_x_;
  double observation_y_;
  double px_;
  double py_;
  double pz_;
};

/**
 * @brief   Auto-diff   ReprojectionError struct
 */

struct ReprojectionError {
  ReprojectionError(const CameraInt &cam_int, double observed_x,
                    double observed_y, double px, double py, double pz)
      : cam_int_(cam_int),
        obs_x_(observed_x),
        obs_y_(observed_y),
        px_(px),
        py_(py),
        pz_(pz) {}

  template <typename T>
  bool operator()(const T *const camera, T *residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    T point[3] = {T(px_), T(py_), T(pz_)};

    ceres::AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    T predicted_x = xp * cam_int_.fx + cam_int_.cx;
    T predicted_y = yp * cam_int_.fy + cam_int_.cy;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - obs_x_;
    residuals[1] = predicted_y - obs_y_;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *create(const CameraInt &cam_int,
                                     const double obs_x_, const double obs_y_,
                                     const double px, const double py,
                                     const double pz) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6>(
        new ReprojectionError(cam_int, obs_x_, obs_y_, px, py, pz)));
  }

  CameraInt cam_int_;
  double obs_x_;
  double obs_y_;

  double px_;
  double py_;
  double pz_;
};
