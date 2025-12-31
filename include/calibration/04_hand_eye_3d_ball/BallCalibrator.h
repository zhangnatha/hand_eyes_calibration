#pragma once

#include "calibration/hand_eye_3d_base/RobotCameraCalibrator3D.h"
#include "common/Pose3d.h"
#include <string>
#include <vector>

/**
 * @brief 3D 相机手眼标定 - 基于标定球
 */
class BallCalibrator {
public:
  BallCalibrator() {}
  virtual ~BallCalibrator() {}

  /**
   * @brief 从文件加载数据并执行标定
   * @param data_file [in] 数据文件路径 (包含机器人位姿和标定球球心坐标)
   * @param type [in] EIH/ETH
   * @return RMSE 误差，失败返回 -1
   */
  double calibrateFromFile(const std::string &data_file,
                           CalibrationType3D type);

  /**
   * @brief 获取标定结果位姿
   * @return Pose3D 格式的结果
   */
  Pose3D getResultPose() const { return result_pose_; }

  /**
   * @brief 获取标定误差列表
   */
  std::vector<double> getErrors() const { return errors_; }

  /**
   * @brief 设置欧拉角类型
   */
  void setRPYType(RPY::RPYType type,
                  RPY::ReferenceType ref = RPY::ReferenceType::EXTRINSIC) {
    rpy_type_ = type;
    ref_type_ = ref;
  }

  /**
   * @brief 设置机器人位姿列表
   */
  void setRobotPoses(const std::vector<Pose3D> &poses) {
    robot_poses_p3d_ = poses;
  }

  /**
   * @brief 设置标记点列表 (球心坐标)
   */
  void setMarkPoints(const std::vector<Eigen::Vector3d> &points) {
    mark_points_ = points;
  }

  /**
   * @brief 设置标定类型
   */
  void setCalibrationType(CalibrationType3D type) { calib_type_ = type; }

  /**
   * @brief 执行标定
   * @return 成功返回 true
   */
  bool runCalibration();

  /**
   * @brief 获取输入的机器人位姿列表 (4x4 矩阵) - 在 runCalibration 后可用
   */
  std::vector<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>>
  getRobotPoses() const {
    return robot_poses_mat_;
  }

  /**
   * @brief 获取输入的标记点列表
   */
  std::vector<Eigen::Vector3d> getMarkPoints() const { return mark_points_; }

private:
  Pose3D result_pose_;
  CalibrationType3D calib_type_ = CalibrationType3D::ETH;
  std::vector<double> errors_;
  std::vector<Pose3D> robot_poses_p3d_;
  std::vector<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>> robot_poses_mat_;
  std::vector<Eigen::Vector3d> mark_points_;
  RPY::RPYType rpy_type_ = RPY::RPYType::XYZ;
  RPY::ReferenceType ref_type_ = RPY::ReferenceType::EXTRINSIC;
};
