#pragma once

#include <Eigen/Dense>
#include "type.h"

// https://en.cppreference.com/w/cpp/types/numeric_limits
const double SMALL_EPS = std::numeric_limits<double>::min();

inline Mat33d skew(const Eigen::Vector3d& v) {
  Mat33d m;
  m.setZero();
  m(0, 1) = -v(2);
  m(0, 2) = v(1);
  m(1, 2) = -v(0);
  m(1, 0) = v(2);
  m(2, 0) = -v(1);
  m(2, 1) = v(0);
  return m;
}

inline Vec3d toAngleAxis(const Eigen::Quaterniond& quaterd,
                         double* angle = NULL) {
  Eigen::Quaterniond unit_quaternion = quaterd.normalized();
  double n = unit_quaternion.vec().norm();
  double w = unit_quaternion.w();
  double squared_w = w * w;

  double two_atan_nbyw_by_n;
  // Atan-based log thanks to
  //
  // C. Hertzberg et al.:
  // "Integrating Generic Sensor Fusion Algorithms with Sound State
  // Representation through Encapsulation of Manifolds"
  // Information Fusion, 2011

  if (n < SMALL_EPS) {
    // If quaternion is normalized and n=1, then w should be 1;
    // w=0 should never happen here!
    assert(fabs(w) > SMALL_EPS);

    two_atan_nbyw_by_n = 2. / w - 2. * (n * n) / (w * squared_w);
  } else {
    if (fabs(w) < SMALL_EPS) {
      if (w > 0) {
        two_atan_nbyw_by_n = M_PI / n;
      } else {
        two_atan_nbyw_by_n = -M_PI / n;
      }
    }
    two_atan_nbyw_by_n = 2 * atan(n / w) / n;
  }
  if (angle != NULL) *angle = two_atan_nbyw_by_n * n;
  return two_atan_nbyw_by_n * unit_quaternion.vec();
}

inline Eigen::Quaterniond toQuaterniond(const Eigen::Vector3d& v3d,
                                        double* angle = NULL) {
  double theta = v3d.norm();
  if (angle != NULL) *angle = theta;
  double half_theta = 0.5 * theta;

  double imag_factor;
  double real_factor = cos(half_theta);
  if (theta < SMALL_EPS) {
    double theta_sq = theta * theta;
    double theta_po4 = theta_sq * theta_sq;
    imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4;
  } else {
    double sin_half_theta = sin(half_theta);
    imag_factor = sin_half_theta / theta;
  }

  return Eigen::Quaterniond(real_factor, imag_factor * v3d.x(),
                            imag_factor * v3d.y(), imag_factor * v3d.z());
}

class SE3 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 protected:
  Eigen::Quaterniond r_;
  Eigen::Vector3d t_;

 public:
  SE3() {
    r_.setIdentity();
    t_.setZero();
  }
  SE3(const Mat33d& rotation, const Vec3d& t)
      : r_(Eigen::Quaterniond(rotation)), t_(t) {
    normalizeRotation();
  }
  SE3(const Eigen::Quaterniond& q, const Vec3d& t) : r_(q), t_(t) {
    normalizeRotation();
  }

  inline const Vec3d& translation() const { return t_; }

  inline Vec3d& translation() { return t_; }

  inline void setTranslation(const Vec3d& t) { t_ = t; }

  inline const Eigen::Quaterniond& rotation() const { return r_; }

  inline Eigen::Quaterniond& rotation() { return r_; }

  void setRotation(const Eigen::Quaterniond& r) { r_ = r; }

  inline SE3 operator*(const SE3& tr2) const {
    SE3 result(*this);
    result.t_ += r_ * tr2.t_;
    result.r_ *= tr2.r_;
    result.normalizeRotation();
    return result;
  }

  inline SE3& operator*=(const SE3& tr2) {
    t_ += r_ * tr2.t_;
    r_ *= tr2.r_;
    normalizeRotation();
    return *this;
  }

  inline Vec3d operator*(const Vec3d& v) const { return t_ + r_ * v; }

  inline SE3 inverse() const {
    SE3 ret;
    ret.r_ = r_.conjugate();
    ret.t_ = ret.r_ * (t_ * -1.);
    return ret;
  }

  inline double operator[](int i) const {
    assert(i < 7);
    if (i < 4) return r_.coeffs()[i];
    return t_[i - 4];
  }

  inline Vec7d toVector() const {
    Vec7d v;
    v.head<4>() = Eigen::Vector4d(r_.coeffs());
    v.tail<3>() = t_;
    return v;
  }

  inline void fromVector(const Vec7d& v) {
    r_ = Eigen::Quaterniond(v[3], v[0], v[1], v[2]);
    t_ = Eigen::Vector3d(v[4], v[5], v[6]);
  }

  Vec6d log() const {
    Vec6d res;

    double theta;
    res.head<3>() = toAngleAxis(r_, &theta);

    Eigen::Matrix3d Omega = skew(res.head<3>());
    Eigen::Matrix3d V_inv;
    if (theta < SMALL_EPS) {
      V_inv = Eigen::Matrix3d::Identity() - 0.5 * Omega +
              (1. / 12.) * (Omega * Omega);
    } else {
      V_inv = (Eigen::Matrix3d::Identity() - 0.5 * Omega +
               (1 - theta / (2 * tan(theta / 2))) / (theta * theta) *
                   (Omega * Omega));
    }

    res.tail<3>() = V_inv * t_;

    return res;
  }

  Eigen::Vector3d map(const Eigen::Vector3d& xyz) const {
    return r_ * xyz + t_;
  }

  static SE3 exp(const Vec6d& update) {
    Vec3d omega(update.data());
    Vec3d upsilon(update.data() + 3);

    double theta;
    Mat33d Omega = skew(omega);

    Eigen::Quaterniond R = toQuaterniond(omega, &theta);
    Mat33d V;
    if (theta < SMALL_EPS) {
      V = R.matrix();
    } else {
      Mat33d Omega2 = Omega * Omega;

      V = (Eigen::Matrix3d::Identity() +
           (1 - cos(theta)) / (theta * theta) * Omega +
           (theta - sin(theta)) / (pow(theta, 3)) * Omega2);
    }
    return SE3(R, V * upsilon);
  }

  Eigen::Matrix<double, 6, 6, Eigen::ColMajor> adj() const {
    Mat33d R = r_.toRotationMatrix();
    Eigen::Matrix<double, 6, 6, Eigen::ColMajor> res;
    res.block(0, 0, 3, 3) = R;
    res.block(3, 3, 3, 3) = R;
    res.block(3, 0, 3, 3) = skew(t_) * R;
    res.block(0, 3, 3, 3) = Mat33d::Zero(3, 3);
    return res;
  }

  Eigen::Matrix<double, 4, 4, Eigen::ColMajor> to_homogeneous_matrix() const {
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> homogeneous_matrix;
    homogeneous_matrix.setIdentity();
    homogeneous_matrix.block(0, 0, 3, 3) = r_.toRotationMatrix();
    homogeneous_matrix.col(3).head(3) = translation();

    return homogeneous_matrix;
  }

  void normalizeRotation() {
    if (r_.w() < 0) {
      r_.coeffs() *= -1;
    }
    r_.normalize();
  }
};
