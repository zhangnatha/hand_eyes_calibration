#include "calibration/05_hand_eye_3d_board_img/BoardImageCalibrator.h"
#include <algorithm>
#include <iostream>

Pose3D BoardImageCalibrator::extrinsicCalibrate(
    const CalibConfig &cfg, const std::vector<std::string> imgs,
    const std::vector<Pose3D> &robot_poses, CalibrationType3D calib_type) {
  setConfig(cfg);
  setImageFiles(imgs);
  setRobotPoses(robot_poses);
  setCalibrationType(calib_type);

  if (runCalibration()) {
    return result_pose_;
  }
  return Pose3D();
}

bool BoardImageCalibrator::runCalibration() {
  cv::FileStorage fs(cfg_.xml_intrinsic_1st, cv::FileStorage::READ);
  cv::Mat K, dist;
  fs["K"] >> K;
  fs["distortion"] >> dist;
  fs.release();
  if (K.empty()) {
    std::cerr << "未找到内参！请先运行 intrinsicCalibrate\n";
    return false;
  }
  std::vector<cv::Mat> all_images;
  std::vector<std::vector<cv::Point3f>> worlds_list_second;
  std::vector<std::vector<cv::Point2f>> pixels_list_second;
  cv::Size imgSize;
  int valid = 0;
  for (size_t i = 0; i < img_files_.size(); ++i) {
    cv::Mat raw = cv::imread(img_files_[i]);
    if (raw.empty())
      continue;
    cv::Mat input_image_in, input_image_out, output_corners_image;
    input_image_in = raw.clone();
    cv::undistort(input_image_in, input_image_out, K, dist);
    all_images.push_back(input_image_out);
    if (imgSize == cv::Size())
      imgSize = raw.size();
    std::vector<cv::Point2f> pixel_corners;
    std::vector<cv::Point3f> world_corners;
    int ret = IntrinsicCalibrator::detectCalibBoard(
        input_image_out, output_corners_image, pixel_corners, world_corners,
        cfg_, 0);
    if (ret == 1) {
      worlds_list_second.push_back(world_corners);
      pixels_list_second.push_back(pixel_corners);
      valid++;
    }
    std::cout << "\r进度: " << i + 1 << "/" << img_files_.size()
              << " 有效: " << valid << std::flush;
  }
  std::cout << "\n";
  if (valid < 5) {
    std::cerr << "至少 5 张！\n";
    return false;
  }
  cv::Mat K_second, dis_second;
  std::vector<cv::Mat> rvecs, tvecs;
  double rms =
      cv::calibrateCamera(worlds_list_second, pixels_list_second, imgSize,
                          K_second, dis_second, rvecs, tvecs, 0);
  std::cout << "第二次标定成功！RMS = " << rms << "\n";
  std::cout << "内参: \n" << K_second << "\n";
  std::cout << "畸变: \n" << dis_second << "\n";
  cv::FileStorage fs_2nd(cfg_.xml_intrinsic_2nd, cv::FileStorage::WRITE);
  fs_2nd << "K" << K_second << "distortion" << dis_second << "pattern"
         << (int)cfg_.pattern;
  fs_2nd.release();

  std::vector<Pose3D> success_poses;
  std::vector<Vector3D> success_marks;
  marker_points_.assign(robot_poses_.size(), Vector3D(0, 0, 0));
  marker_success_.assign(robot_poses_.size(), false);

  for (size_t i = 0; i < robot_poses_.size(); ++i) {
    if (i >= all_images.size())
      continue;

    cv::Mat output_corners_image;
    std::vector<cv::Point2f> pixel_corners;
    std::vector<cv::Point3f> world_corners;
    int ret = IntrinsicCalibrator::detectCalibBoard(
        all_images[i], output_corners_image, pixel_corners, world_corners, cfg_,
        1);
    if (ret == 1) {
      cv::Mat rvec, tvec;
      cv::solvePnP(world_corners, pixel_corners, K_second, dis_second, rvec,
                   tvec);
      Vector3D point_xyz_camera;
      point_xyz_camera[0] = tvec.at<double>(0, 0);
      point_xyz_camera[1] = tvec.at<double>(0, 1);
      point_xyz_camera[2] = tvec.at<double>(0, 2);

      success_marks.push_back(point_xyz_camera);
      success_poses.push_back(robot_poses_[i]);

      marker_points_[i] = point_xyz_camera;
      marker_success_[i] = true;
    }
  }

  if (success_poses.empty())
    return false;

  RobotCameraCalibrator3D calibrator;
  calibrator.setRPYType(rpy_type_, ref_type_);

  std::vector<Eigen::Vector3d> marks_eigen;
  for (const auto &m : success_marks) {
    marks_eigen.push_back(m.eigen());
  }

  Eigen::Matrix4d transform;
  Eigen::Vector3d tcp;
  double result = calibrator.calibrate(marks_eigen, success_poses, calib_type_,
                                       transform, tcp);

  if (std::isnan(result)) {
    return false;
  } else {
    auto raw_errors = calibrator.getCalibrationError();
    errors_.assign(robot_poses_.size(), -1.0);
    int error_idx = 0;
    for (size_t i = 0; i < robot_poses_.size(); ++i) {
      if (marker_success_[i] && error_idx < raw_errors.size()) {
        errors_[i] = raw_errors[error_idx++];
      }
    }

    result_pose_ = Pose3D(Transform3D(transform));
    return true;
  }
}
