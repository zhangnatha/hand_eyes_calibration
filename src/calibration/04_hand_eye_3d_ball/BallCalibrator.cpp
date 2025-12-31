#include "calibration/04_hand_eye_3d_ball/BallCalibrator.h"
#include <fstream>
#include <iostream>
#include <sstream>

double BallCalibrator::calibrateFromFile(const std::string &data_file,
                                         CalibrationType3D type) {
  std::ifstream f_data(data_file);
  if (!f_data.is_open()) {
    return -1.0;
  }

  mark_points_.clear();
  robot_poses_p3d_.clear();

  std::string line;
  while (std::getline(f_data, line)) {
    if (line.empty())
      continue;
    std::stringstream ss(line);
    std::string segment;
    std::vector<double> val;
    while (std::getline(ss, segment, ',')) {
      try {
        val.push_back(std::stod(segment));
      } catch (...) {
        continue;
      }
    }
    if (val.size() >= 9) {
      robot_poses_p3d_.push_back(
          Pose3D(val[0], val[1], val[2], val[3], val[4], val[5]));
      mark_points_.push_back(Eigen::Vector3d(val[6], val[7], val[8]));
    }
  }
  f_data.close();

  calib_type_ = type;

  if (runCalibration()) {
    return errors_.empty() ? 0.0 : errors_[0]; // Simplified return
  }

  return -1.0;
}

bool BallCalibrator::runCalibration() {
  if (mark_points_.empty() || robot_poses_p3d_.empty()) {
    return false;
  }

  RobotCameraCalibrator3D robot_calib;
  robot_calib.setRPYType(rpy_type_, ref_type_);
  Eigen::Matrix4d transform;
  Eigen::Vector3d tcp;
  double error = robot_calib.calibrate(mark_points_, robot_poses_p3d_,
                                       calib_type_, transform, tcp);

  if (error >= 0) {
    result_pose_ = Pose3D(Transform3D(transform));
    errors_ = robot_calib.getCalibrationError();

    robot_poses_mat_.clear();
    for (auto &p : robot_poses_p3d_) {
      robot_poses_mat_.push_back(p.toTransform3D().eigen());
    }
    return true;
  }

  return false;
}
