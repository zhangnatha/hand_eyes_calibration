#include "calibration/06_hand_eye_3d_board_cloud/BoardCloudCalibrator.h"
#include "calibration/01_intrinsic/IntrinsicCalibrator.h"
#include "calibration/hand_eye_3d_base/RobotCameraCalibrator3D.h"
#include "common/CoordinateTransformer.h"
#include "common/pc_LUT.h"
#include <iostream>
// #include <pcl/io/pcd_io.h>
#include <Eigen/SVD>
#include <opencv2/core/eigen.hpp>

/**
 * @brief 3D-3D 刚体变换求解 (SVD 算法)
 * 参考 CalibrationRobotCamera3DChess::getRigidTrans3D
 */
static Eigen::Vector3d
getRigidOrigin(const std::vector<cv::Point3f> &srcPoints,
               const std::vector<cv::Point3f> &dstPoints) {
  int n = srcPoints.size();
  if (n < 3)
    return Eigen::Vector3d(0, 0, 0);

  Eigen::Vector3d centerSrc(0, 0, 0), centerDst(0, 0, 0);
  for (int i = 0; i < n; ++i) {
    centerSrc +=
        Eigen::Vector3d(srcPoints[i].x, srcPoints[i].y, srcPoints[i].z);
    centerDst +=
        Eigen::Vector3d(dstPoints[i].x, dstPoints[i].y, dstPoints[i].z);
  }
  centerSrc /= n;
  centerDst /= n;

  Eigen::MatrixXd srcMat(3, n), dstMat(3, n);
  for (int i = 0; i < n; ++i) {
    srcMat.col(i) =
        Eigen::Vector3d(srcPoints[i].x, srcPoints[i].y, srcPoints[i].z) -
        centerSrc;
    dstMat.col(i) =
        Eigen::Vector3d(dstPoints[i].x, dstPoints[i].y, dstPoints[i].z) -
        centerDst;
  }

  Eigen::Matrix3d H = srcMat * dstMat.transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d R = V * U.transpose();

  if (R.determinant() < 0) {
    V.col(2) *= -1;
    R = V * U.transpose();
  }

  Eigen::Vector3d t = centerDst - R * centerSrc;
  // 返回标定板原点 (0,0,0) 在相机坐标系下的位置，即平移向量 t
  return t;
}

/**
 * @brief 获取邻域像素坐标
 * 参考 CalibrationRobotCamera3DChess::computeNeighbor
 */
static std::vector<cv::Point2i> getNeighborPixels(int x, int y) {
  return {{x, y},     {x, y - 1}, {x - 1, y - 1}, {x + 1, y - 1}, {x - 1, y},
          {x + 1, y}, {x, y + 1}, {x - 1, y + 1}, {x + 1, y + 1}};
}

Pose3D BoardCloudCalibrator::extrinsicCalibrateWithCloud(
    const CalibConfig &cfg, const std::vector<std::string> &imgs,
    const std::vector<std::string> &cloud_luts,
    const std::vector<Pose3D> &robot_poses, CalibrationType3D type) {
  setConfig(cfg);
  setImageFiles(imgs);
  setCloudLuts(cloud_luts);
  setRobotPoses(robot_poses);
  setCalibrationType(type);

  if (runCalibration()) {
    return result_pose_;
  }
  return Pose3D();
}

bool BoardCloudCalibrator::runCalibration() {
  if (img_files_.size() != cloud_luts_.size() ||
      img_files_.size() != robot_poses_.size() ||
      (!cloud_files_.empty() && img_files_.size() != cloud_files_.size())) {
    std::cerr << "输入数据数量不匹配！(Images: " << img_files_.size()
              << ", LUTs: " << cloud_luts_.size()
              << ", Clouds: " << cloud_files_.size()
              << ", Poses: " << robot_poses_.size() << ")\n";
    return false;
  }

  std::vector<Vector3D> success_marks;
  std::vector<Pose3D> success_poses;
  marker_points_.assign(robot_poses_.size(), Vector3D(0, 0, 0));
  marker_success_.assign(robot_poses_.size(), false);

  IntrinsicCalibrator detector;
  PointCloudRegLUTHandler lut_handler;

  for (size_t i = 0; i < img_files_.size(); ++i) {
    cv::Mat raw = cv::imread(img_files_[i]);
    if (raw.empty())
      continue;

    // 1. 加载对应的点云查找表 (Binary 格式)
    std::shared_ptr<PointCloudRegLUT> lut;
    try {
      lut = std::make_shared<PointCloudRegLUT>(
          lut_handler.loadFromFile(cloud_luts_[i], true));
    } catch (const std::exception &e) {
      std::cerr << "无法加载 LUT: " << cloud_luts_[i] << " - " << e.what()
                << "\n";
      continue;
    }

    std::vector<cv::Point2f> pixel_corners;
    std::vector<cv::Point3f> world_corners;
    cv::Mat output_img;

    // 3. 在 2D 图像中提取标定板角点
    if (detector.detectCalibBoard(raw, output_img, pixel_corners, world_corners,
                                  cfg_, 0) == 1) {

      std::vector<cv::Point3f> corners_in_cam;
      std::vector<cv::Point3f> corners_in_chess_valid;

      for (size_t j = 0; j < pixel_corners.size(); ++j) {
        int x = std::round(pixel_corners[j].x);
        int y = std::round(pixel_corners[j].y);

        // --- 邻域像素提取与深度平均 ---
        auto neighbors = getNeighborPixels(x, y);
        cv::Point3d refined_p3d(0, 0, 0);
        int valid_neighbors = 0;

        for (const auto &nb : neighbors) {
          if (nb.y >= 0 && static_cast<size_t>(nb.y) < lut->size() &&
              nb.x >= 0 && static_cast<size_t>(nb.x) < (*lut)[nb.y].size()) {
            int point_index = (*lut)[nb.y][nb.x];
            if (point_index >= 0) {
              // TODO: 此处后续接入点云数据
            }
          }
        }

        if (valid_neighbors > 0) {
          corners_in_cam.push_back(cv::Point3f(
              refined_p3d.x / valid_neighbors, refined_p3d.y / valid_neighbors,
              refined_p3d.z / valid_neighbors));
          corners_in_chess_valid.push_back(world_corners[j]);
        }
      }

      // 4. 对齐参考代码：利用 SVD 拟合全部角点得到 Board Pose 的平移向量
      if (corners_in_cam.size() >= 10) {
        Eigen::Vector3d origin =
            getRigidOrigin(corners_in_chess_valid, corners_in_cam);
        Vector3D marker_pt(origin);
        success_marks.push_back(marker_pt);
        success_poses.push_back(robot_poses_[i]);

        marker_points_[i] = marker_pt;
        marker_success_[i] = true;
      }
    }
  }

  if (success_marks.size() < 5) {
    std::cerr << "有效标定点不足 5 个！\n";
    return false;
  }

  RobotCameraCalibrator3D calibrator;
  calibrator.setRPYType(rpy_type_, ref_type_);

  std::vector<Eigen::Vector3d> marks_eigen;
  for (const auto &m : success_marks)
    marks_eigen.push_back(m.eigen());

  Eigen::Matrix4d transform;
  Eigen::Vector3d tcp;
  double result = calibrator.calibrate(marks_eigen, success_poses, calib_type_,
                                       transform, tcp);

  if (std::isnan(result) || result < 0) {
    std::cerr << "基于点云的手眼标定计算失败！\n";
    return false;
  }

  result_pose_ = Pose3D(Transform3D(transform));

  auto raw_errors = calibrator.getCalibrationError();
  errors_.assign(robot_poses_.size(), -1.0);
  int error_idx = 0;
  for (size_t i = 0; i < robot_poses_.size(); ++i) {
    if (marker_success_[i] && error_idx < raw_errors.size()) {
      errors_[i] = raw_errors[error_idx++];
    }
  }
  return true;
}
