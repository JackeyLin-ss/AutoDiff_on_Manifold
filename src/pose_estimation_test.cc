#include "timer.h"
#include "viewer/viewer.h"
#include <string>
#include <thread>

double fx = 458.65399169921875;
double fy = 457.29598999023438;
double cx = 367.21499633789062;
double cy = 248.375;

// current frame pose
// [0.70724761, 0.22310899, -0.67083764, -0.98894656;
//  -0.25269556, 0.96598929, 0.054860607, 0.1883817;
//  0.66026187, 0.13071765, 0.73957229, 0.49688229;
//  0, 0, 0, 1]

int main(int argc, char **argv) {

  if (argc != 2) {
    std::cout << " args error " << std::endl;
    return 1;
  }

  std::string str_feature_and_mappoints = std::string(argv[1]);

  Viewer viewer;
  viewer.LoadMappoints(str_feature_and_mappoints);

  Mat44t ground_truth_pose;
  ground_truth_pose << 0.70724761, 0.22310899, -0.67083764, -0.98894656,
      -0.25269556, 0.96598929, 0.054860607, 0.1883817, 0.66026187, 0.13071765,
      0.73957229, 0.49688229, 0, 0, 0, 1;

  // Setup Viewer
  viewer.SetGroundTruthPose(ground_truth_pose);

  std::thread *viewer_thread = new std::thread(&Viewer::Run, &viewer);
  viewer_thread->detach();

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 1;
}
