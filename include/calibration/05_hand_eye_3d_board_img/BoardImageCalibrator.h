#pragma once

#include "calibration/01_intrinsic/IntrinsicCalibrator.h"
#include "calibration/hand_eye_3d_base/RobotCameraCalibrator3D.h"
#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief 基于标定板图像的 3D 手眼标定类
 */
class BoardImageCalibrator {
public:
  BoardImageCalibrator() {}
  virtual ~BoardImageCalibrator() {}

  /**
   * @brief 设置标定配置
   */
  void setConfig(const CalibConfig &cfg) { cfg_ = cfg; }

  /**
   * @brief 设置图像文件列表
   */
  void setImageFiles(const std::vector<std::string> &imgs) {
    img_files_ = imgs;
  }

  /**
   * @brief 设置机器人位姿列表
   */
  void setRobotPoses(const std::vector<Pose3D> &poses) { robot_poses_ = poses; }

  /**
   * @brief 设置标定类型
   */
  void setCalibrationType(CalibrationType3D type) { calib_type_ = type; }

  /**
   * @brief 执行相机外参标定 (手眼标定)
   */
  Pose3D extrinsicCalibrate(const CalibConfig &cfg,
                            const std::vector<std::string> imgs,
                            const std::vector<Pose3D> &robot_poses,
                            CalibrationType3D calib_type);

  /**
   * @brief 执行标定
   * @return 成功返回 true
   */
  virtual bool runCalibration();

  /**
   * @brief 获取标定结果
   */
  Pose3D getResultPose() const { return result_pose_; }

  /**
   * @brief 获取标定误差列表
   */
  std::vector<double> getErrors() const { return errors_; }

  /**
   * @brief 获取标记点列表 (相机坐标系)
   */
  std::vector<Vector3D> getMarkerPoints() const { return marker_points_; }

  /**
   * @brief 获取每组数据是否成功检测到标记点
   */
  std::vector<bool> getMarkerSuccess() const { return marker_success_; }

  /**
   * @brief 设置欧拉角类型
   */
  void setRPYType(RPY::RPYType type,
                  RPY::ReferenceType ref = RPY::ReferenceType::EXTRINSIC) {
    rpy_type_ = type;
    ref_type_ = ref;
  }

protected:
  CalibConfig cfg_;
  std::vector<std::string> img_files_;
  std::vector<Pose3D> robot_poses_;
  CalibrationType3D calib_type_ = CalibrationType3D::ETH;
  Pose3D result_pose_;
  std::vector<double> errors_;
  std::vector<Vector3D> marker_points_; // 存储每个位姿对应的标记点
  std::vector<bool> marker_success_;    // 存储每个位姿是否检测成功

  RPY::RPYType rpy_type_ = RPY::RPYType::XYZ;
  RPY::ReferenceType ref_type_ = RPY::ReferenceType::EXTRINSIC;
};
