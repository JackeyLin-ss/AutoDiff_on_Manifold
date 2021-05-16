#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <thread>
#include <random>
#include "pose_estimator.h"
#include "re_projection_error.h"
#include "timer.h"
#include "type.h"
#include "viewer/viewer.h"

using ceres::Problem;
using ceres::AutoDiffCostFunction;
using ceres::SizedCostFunction;

Vec6d isoMat2Vector(const Eigen::Matrix4d pose) {
  Eigen::Matrix3d rotation = pose.block(0, 0, 3, 3);
  Eigen::AngleAxisd angle_axis(rotation);
  Eigen::Vector3d rv = angle_axis.angle() * angle_axis.axis();
  Vec6d v;
  v << rv.x(), rv.y(), rv.z(), pose(0, 3), pose(1, 3), pose(2, 3);
  return v;
}

Eigen::Matrix4d vector2IsoMat(const Vec6d &v) {
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

void addNoise(Vec6d &v, double rotation_noise, double translation_noise) {
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
  bool use_analytic = false;

  std::cout << kColorGreen << (use_analytic ? " Use Analytic Derivatives "
                                            : " Use Automatic Derivatives")
            << kColorReset << std::endl;

  std::string str_map_points = std::string(argv[1]);
  std::string str_observation = std::string(argv[2]);

  Mat44d ground_truth_pose;
  ground_truth_pose << -0.38773203, -0.29730779, 0.872509, -1.8075688,
      0.29847804, 0.85506284, 0.42400289, 0.0065075331, -0.87210935, 0.42482427,
      -0.24279539, 0.75284511, 0, 0, 0, 1;

  auto init_parameter = isoMat2Vector(ground_truth_pose);

  // add noise to ground_truth
  addNoise(init_parameter, 0.1, 0.2);

  // Setup Viewer

  Viewer viewer;
  viewer.LoadMappoints(str_map_points);
  viewer.SetGroundTruthPose(ground_truth_pose);
  viewer.SetInitialPose(vector2IsoMat(init_parameter));
  std::thread *viewer_thread = new std::thread(&Viewer::Run, &viewer);
  viewer_thread->detach();

  // start solving pose estimation problem

  std::shared_ptr<PoseEstimationProblem> pose_estimation =
      std::make_shared<PoseEstimationProblem>();
  pose_estimation->loadFile(str_observation);
  pose_estimation->setInitialParameter(init_parameter.data());

  ceres::Problem problem;

  // camera intrinsic parameters
  double fx = 458.65399169921875;
  double fy = 457.29598999023438;
  double cx = 367.21499633789062;
  double cy = 248.375;
  CameraInt cam_int(fx, fy, cx, cy);

  if (use_analytic) {
    problem.AddParameterBlock(pose_estimation->mutable_camera_pose(), 6,
                              new PoseSE3Parameterization());
  } else {
    problem.AddParameterBlock(pose_estimation->mutable_camera_pose(), 6);
  }
  for (int i = 0; i < pose_estimation->num_observations(); ++i) {
    auto point = pose_estimation->point_position_for_observation(i);
    auto observation = pose_estimation->observation(i);

    if (use_analytic) {
      ceres::CostFunction *cost_function =
          new ReprojectionErrorSE3(cam_int, observation.x(), observation.y(),
                                   point.x(), point.y(), point.z());
      problem.AddResidualBlock(cost_function, NULL,
                               pose_estimation->mutable_camera_pose());
    } else  // auto-diff
    {
      ceres::CostFunction *cost_function =
          ReprojectionError::create(cam_int, observation.x(), observation.y(),
                                    point[0], point[1], point[2]);
      problem.AddResidualBlock(cost_function, nullptr /* squared loss */,
                               pose_estimation->mutable_camera_pose());
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  // display optimized result
  auto final_result = pose_estimation->mutable_camera_pose();
  Eigen::Map<Vec6d> final_result_vector = Eigen::Map<Vec6d>(final_result);
  viewer.SetEstimatePose(vector2IsoMat(final_result_vector));

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 1;
}
