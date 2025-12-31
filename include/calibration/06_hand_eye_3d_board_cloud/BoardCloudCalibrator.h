#pragma once

#include "calibration/05_hand_eye_3d_board_img/BoardImageCalibrator.h"

/**
 * @brief 3D 相机手眼标定 - 基于标定板图像与点云
 * 与 BoardImageCalibrator 的区别在于：
 * BoardImageCalibrator 通过 PnP 结合内参计算 3D 点，
 * 而 BoardCloudCalibrator 直接从相机 2D 点到 3D 点的查找表获取真实的 (x, y, z)
 * 点云数据。
 */
class BoardCloudCalibrator : public BoardImageCalibrator {
public:
  BoardCloudCalibrator() {}
  virtual ~BoardCloudCalibrator() {}

  /**
   * @brief 设置点云查找表文件列表
   */
  void setCloudLuts(const std::vector<std::string> &luts) {
    cloud_luts_ = luts;
  }

  /**
   * @brief 设置点云文件列表
   */
  void setCloudFiles(const std::vector<std::string> &clouds) {
    cloud_files_ = clouds;
  }

  /**
   * @brief 执行基于点云查找表的外参标定
   */
  Pose3D extrinsicCalibrateWithCloud(const CalibConfig &cfg,
                                     const std::vector<std::string> &imgs,
                                     const std::vector<std::string> &cloud_luts,
                                     const std::vector<Pose3D> &robot_poses,
                                     CalibrationType3D type);

  /**
   * @brief 执行标定 (重写基类)
   */
  bool runCalibration() override;

protected:
  std::vector<std::string> cloud_luts_;
  std::vector<std::string> cloud_files_;
};
