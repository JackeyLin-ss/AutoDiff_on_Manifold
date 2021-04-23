#include "viewer.h"
#include <fstream>
#include <thread>
Viewer::Viewer() {
  estimated_pose_ = Mat44t::Identity();
  ground_truth_pose_ = Mat44t::Identity();
}

Viewer::~Viewer() {}

void Viewer::Run() {
  pangolin::CreateWindowAndBind("Map Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  double mViewpointF = 2000;
  double view_point_x = -10;
  double view_point_y = -10;
  double view_point_z = -10;
  const int kUiWidth = 175;

  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(kUiWidth));
  pangolin::Var<bool> menu_draw_groundtruth_pose("menu.DrawGroundTruth", true,
                                                 true);
  pangolin::Var<bool> menu_draw_estimate_pose("menu.DrawEstimatedPose", true,
                                              true);
  pangolin::Var<bool> menu_draw_initial_pose("menu.DrawInitialPose", true,
                                             true);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389,
                                 0.1, 1000),
      pangolin::ModelViewLookAt(view_point_x, view_point_y, view_point_z,
                                view_point_x + 13, view_point_y + 12,
                                view_point_z + 12, 0.0, 0.0, 1.0));

  pangolin::View &d_cam =
      pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0 / 768.0)
          .SetHandler(new pangolin::Handler3D(s_cam));

  Eigen::Matrix4d m;
  m << -0.0612103, -0.216861, 0.974281, -0.873631, -0.407822, -0.885482,
      -0.222717, -0.760422, 0.911007, -0.410967, -0.0342404, -31.595, 0, 0, 0,
      1;
  pangolin::OpenGlMatrix om(m);
  s_cam.SetModelViewMatrix(om);

  while (true) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);

    glClearColor(0.0156f, 0.1019f, 0.231f, 0.0f);

    // draw map
    DrawMapPoints();
    //    DrawAxis();

    if (menu_draw_estimate_pose) DrawEstimatePose();

    if (menu_draw_groundtruth_pose) DrawGroundTruthPose();

    if (menu_draw_initial_pose) DrawInitialPose();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pangolin::FinishFrame();
  }
}

void Viewer::LoadMappoints(const std::string &files) {
  std::ifstream fin;
  fin.open(files);
  if (!fin.is_open()) {
    std::cerr << " file " << files << " is not found " << std::endl;
    return;
  }

  std::string line;
  int point_num = 0;
  std::getline(fin, line);
  sscanf(line.c_str(), "%d", &point_num);

  for (int i = 0; i < point_num; i++) {
    std::getline(fin, line);
    float x, y, z;
    sscanf(line.c_str(), "%f %f %f", &x, &y, &z);
    points_.push_back(Vec3_t(x, y, z));
    auto color = MakeJet3B((y + 2) / 4.);
    colors_.push_back(
        Vec3_t(color[0] / 255., color[1] / 255., color[2] / 255.));
  }
  fin.close();
}

void Viewer::SetGroundTruthPose(const Mat44t &pose) {
  std::lock_guard<std::mutex> lock(mutex_pose_);
  ground_truth_pose_ = pose;
}

void Viewer::SetEstimatePose(const Mat44t &pose) {
  std::lock_guard<std::mutex> lock(mutex_pose_);
  estimated_pose_ = pose;
}

void Viewer::SetInitialPose(const Mat44t &pose) {
  std::lock_guard<std::mutex> lock(mutex_pose_);
  initial_pose_ = pose;
}

void Viewer::DrawAxis() {
  glColor3f(1, 0, 0);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(10, 0, 0);
  glColor3f(0, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 10, 0);
  glColor3f(0, 0, 1);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, 10);
  glEnd();
}

void Viewer::DrawMapPoints() {
  glPointSize(3);
  glColor3f(1, 0, 0);
  glBegin(GL_POINTS);

  for (size_t i = 0; i < colors_.size(); i++) {
    auto p = points_[i];
    auto c = colors_[i];
    glColor3f(c.x(), c.y(), c.z());
    glVertex3f(p.x(), p.y(), p.z());
  }

  glEnd();
}

void Viewer::DrawGroundTruthPose() {
  Mat44t tcw;
  {
    std::unique_lock<std::mutex> lock(mutex_pose_);
    tcw = ground_truth_pose_;
  }
  Mat44t twc = tcw.inverse();
  glPushMatrix();
  glMultMatrixd(twc.data());
  DrawCameraWireframe(1, 0, 0);
  glPopMatrix();
}

void Viewer::DrawEstimatePose() {
  Mat44t tcw;
  {
    std::unique_lock<std::mutex> lock(mutex_pose_);
    tcw = estimated_pose_;
  }
  Mat44t twc = tcw.inverse();
  glPushMatrix();
  glMultMatrixd(twc.data());
  DrawCameraWireframe(0, 1, 0);
  glPopMatrix();
}

void Viewer::DrawInitialPose() {
  Mat44t tcw;
  {
    std::unique_lock<std::mutex> lock(mutex_pose_);
    tcw = initial_pose_;
  }
  Mat44t twc = tcw.inverse();
  glPushMatrix();
  glMultMatrixd(twc.data());
  DrawCameraWireframe(0, 0, 1);
  glPopMatrix();
}

void Viewer::DrawCameraWireframe(float r, float g, float b) {
  glColor3f(r, g, b);
  glLineWidth(2.0);

  glBegin(GL_LINES);

  const float kCameraWidth = 0.4;
  const float kCameraHeight = 0.3;
  const float kCameraDepth = 0.2;

  glVertex3f(kCameraWidth / 2., kCameraHeight / 2., kCameraDepth);
  glVertex3f(kCameraWidth / 2., -kCameraHeight / 2., kCameraDepth);

  glVertex3f(kCameraWidth / 2., -kCameraHeight / 2., kCameraDepth);
  glVertex3f(-kCameraWidth / 2., -kCameraHeight / 2., kCameraDepth);

  glVertex3f(-kCameraWidth / 2., -kCameraHeight / 2., kCameraDepth);
  glVertex3f(-kCameraWidth / 2., kCameraHeight / 2., kCameraDepth);

  glVertex3f(-kCameraWidth / 2., kCameraHeight / 2., kCameraDepth);
  glVertex3f(kCameraWidth / 2., kCameraHeight / 2., kCameraDepth);

  glVertex3f(0, 0, 0);
  glVertex3f(kCameraWidth / 2., kCameraHeight / 2., kCameraDepth);

  glVertex3f(0, 0, 0);
  glVertex3f(kCameraWidth / 2., -kCameraHeight / 2., kCameraDepth);

  glVertex3f(0, 0, 0);
  glVertex3f(-kCameraWidth / 2., -kCameraHeight / 2., kCameraDepth);

  glVertex3f(0, 0, 0);
  glVertex3f(-kCameraWidth / 2., kCameraHeight / 2., kCameraDepth);

  glEnd();
}
