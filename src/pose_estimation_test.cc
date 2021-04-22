#include "timer.h"
#include "viewer/viewer.h"
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <fstream>
#include <string>
#include <thread>

double fx = 458.65399169921875;
double fy = 457.29598999023438;
double cx = 367.21499633789062;
double cy = 248.375;

using Vec6d = Eigen::Matrix<double, 6, 1>;

class PoseEstimationProblem {
public:
  PoseEstimationProblem() { num_parameters_ = 6; }
  ~PoseEstimationProblem() {
    delete[] points_;
    delete[] observations_;
    delete[] parameters_;
  }

  int num_observations() const { return num_observations_; }
  const double *observations() const { return observations_; }

  double *mutable_cameras() { return parameters_; }

  double *observation(int i) const { return observations_ + 2 * i; }

  double *point_position_for_observation(int i) const {
    return points_ + 3 * i;
  }

  bool SetInitialParameter(const double *const data) {
    memcpy(parameters_, data,
           sizeof(double) * static_cast<unsigned long>(num_parameters_));
    return true;
  }

  bool LoadFile(const std::string &filename) {
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

    observations_ = new double[2 * num_observations_];
    points_ = new double[3 * num_points_];
    parameters_ = new double[num_parameters_];

    for (int i = 0; i < num_observations(); ++i) {
      // 3d point
      std::getline(fin, line);
      sscanf(line.c_str(), "%lf %lf %lf", points_ + 3 * i, points_ + 3 * i + 1,
             points_ + 3 * i + 2);

      // 2d feature points;
      std::getline(fin, line);
      sscanf(line.c_str(), "%lf %lf", observations_ + 2 * i,
             observations_ + 2 * i + 1);
    }
    return true;
  }

private:
  // points' position is fixed
  int num_points_;
  int num_observations_;
  int num_parameters_;

  double *points_;
  double *observations_;
  double *parameters_;
};

struct ReprojectionError {
  ReprojectionError(double observed_x, double observed_y, double px, double py,
                    double pz)
      : px_(px), py_(py), pz_(pz), observed_x_(observed_x),
        observed_y_(observed_y) {}

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

    T predicted_x = xp * fx + cx;
    T predicted_y = yp * fy + cy;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x_;
    residuals[1] = predicted_y - observed_y_;

    //    std::cout<< residuals[0] << " " << residuals[1] << std::endl;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *Create(const double observed_x_,
                                     const double observed_y_, const double px,
                                     const double py, const double pz) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6>(
        new ReprojectionError(observed_x_, observed_y_, px, py, pz)));
  }

  double px_;
  double py_;
  double pz_;

  double observed_x_;
  double observed_y_;
};

using ceres::Problem;
using ceres::AutoDiffCostFunction;

Vec6d IsoMat2Vector(const Eigen::Matrix4d pose) {

  Eigen::Matrix3d rotation = pose.block(0, 0, 3, 3);
  Eigen::AngleAxisd angle_axis(rotation);
  Eigen::Vector3d rv = angle_axis.angle() * angle_axis.axis();
  Vec6d v;
  v << rv.x(), rv.y(), rv.z(), pose(0, 3), pose(1, 3), pose(2, 3);
  return v;
}

Eigen::Matrix4d Vector2IsoMat(const Vec6d &v) {
  double angle = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  Eigen::Vector3d axis(v[0], v[1], v[2]);
  axis /= angle;
  Eigen::AngleAxisd angle_axis(angle, axis);

  Eigen::MatrixX4d pose = Eigen::Matrix4d::Identity();
  pose.block(0, 0, 3, 3) = angle_axis.toRotationMatrix();

  pose(0, 3) = v[3];
  pose(1, 3) = v[4];
  pose(2, 3) = v[5];
  return pose;
}

void AddNoise(Vec6d &v, double rotation_noise, double translation_noise) {

  const double mean = 0.0;
  std::default_random_engine generator;
  std::normal_distribution<double> dist_rotation(mean, rotation_noise);
  std::normal_distribution<double> dist_translation(mean, translation_noise);
  for (int i = 0; i < 3; i++) {
    v[i] += dist_rotation(generator);
    v[i + 3] += dist_translation(generator);
  }
}

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cout << " args error " << std::endl;
    return 1;
  }

  std::string str_map_points = std::string(argv[1]);
  std::string str_observation = std::string(argv[2]);

  Mat44t ground_truth_pose;
  ground_truth_pose << -0.38773203, -0.29730779, 0.872509, -1.8075688,
      0.29847804, 0.85506284, 0.42400289, 0.0065075331, -0.87210935, 0.42482427,
      -0.24279539, 0.75284511, 0, 0, 0, 1;

  auto init_parameter = IsoMat2Vector(ground_truth_pose);

  // add noise to ground_truth
  AddNoise(init_parameter, 0.1, 0.2);

  // Setup Viewer

  Viewer viewer;
  viewer.LoadMappoints(str_map_points);
  viewer.SetGroundTruthPose(ground_truth_pose);
  viewer.SetInitialPose(Vector2IsoMat(init_parameter));
  std::thread *viewer_thread = new std::thread(&Viewer::Run, &viewer);
  viewer_thread->detach();

  // start solving pose estimation problem

  std::shared_ptr<PoseEstimationProblem> pose_estimation =
      std::make_shared<PoseEstimationProblem>();
  pose_estimation->LoadFile(str_observation);
  pose_estimation->SetInitialParameter(init_parameter.data());

  const double *observations = pose_estimation->observations();
  ceres::Problem problem;
  for (int i = 0; i < pose_estimation->num_observations(); ++i) {

    double *point = pose_estimation->point_position_for_observation(i);
    ceres::CostFunction *cost_function = ReprojectionError::Create(
        observations[2 * i + 0], observations[2 * i + 1], point[0], point[1],
        point[2]);
    problem.AddResidualBlock(cost_function, nullptr /* squared loss */,
                             pose_estimation->mutable_cameras());
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  // display optimized result
  auto final_result = pose_estimation->mutable_cameras();
  Eigen::Map<Vec6d> final_result_vector = Eigen::Map<Vec6d>(final_result);
  viewer.SetEstimatePose(Vector2IsoMat(final_result_vector));

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 1;
}
