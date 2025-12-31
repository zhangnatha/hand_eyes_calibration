#pragma once

#include "common/CalibrationTypes.h"
#include "common/Pose3d.h"
#include <opencv2/opencv.hpp>
#include <vector>

enum Pattern {
  CHESSBOARD,
  CIRCLES_SYM,
  CIRCLES_ASYM
}; // 标定板图案类型：棋盘格、对称圆、非对称圆

struct CalibConfig {
  Pattern pattern = CHESSBOARD;                          // 标定板图案类型
  CalibrationType3D calib_type = CalibrationType3D::ETH; // 标定类型 (EIH/ETH)
  int cols = 4;                   // 横向点数（棋盘格为内角点数）
  int rows = 5;                   // 纵向点数
  int interval_mm = 20;           // 标定板点间距 (单位: mm)
  float marker_length_mm = 50.0f; // ArUco 码边长 (单位: mm)
  std::string xml_intrinsic_1st =
      "calib_intrinsic_1st.xml"; // 第一相机内参文件名
  std::string xml_intrinsic_2nd =
      "calib_intrinsic_2nd.xml";                     // 第二相机内参文件名
  std::string xml_extrinsic = "calib_extrinsic.xml"; // 外参文件名
};

class IntrinsicCalibrator {
public:
  IntrinsicCalibrator() {}
  virtual ~IntrinsicCalibrator() {}

  /**
   * @brief 执行相机内参标定
   * @param cfg [in] 标定配置参数
   * @param imgs [in] 包含标定板图像的路径列表
   * @return 标定成功返回 true，否则返回 false
   */
  bool intrinsicCalibrate(const CalibConfig &cfg,
                          const std::vector<std::string> imgs);

  /**
   * @brief 检测标定板，返回 2D 像素点 + 3D 世界点
   */
  static int detectCalibBoard(const cv::Mat &input_image,
                              cv::Mat &output_corners_image,
                              std::vector<cv::Point2f> &pixel_corners,
                              std::vector<cv::Point3f> &world_corners,
                              const CalibConfig &cfg, int calib_type);
};
