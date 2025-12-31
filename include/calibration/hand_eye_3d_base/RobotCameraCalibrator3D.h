#ifndef CALIBRATION_ROBOT_CAMERA_3D_H
#define CALIBRATION_ROBOT_CAMERA_3D_H

#include "common/CalibrationTypes.h" // For CalibrationType3D
#include "common/Math.h"             // For RPY
#include "common/Pose3d.h"           // For Pose3D
#include <Eigen/Geometry>
#include <cstdlib>
#include <vector>

class RobotCameraCalibrator3DImpl {
private:
  std::vector<Eigen::Matrix<double, 3, 5>> Pose_Point;
  double errorValue = 0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 3, 4> Eye2Fixed;
  Eigen::Vector3d PointInFixed;

  RobotCameraCalibrator3DImpl() {}
  ~RobotCameraCalibrator3DImpl() {}

  void inputData(
      std::vector<Eigen::Vector3d> &_markPoint,
      std::vector<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>> &_robotPose,
      CalibrationType3D type);

  Eigen::Matrix3d skew(Eigen::Vector3d const &axis);

  Eigen::Matrix<double, 9, 1> update();

  int calib_closedForm();

  int calib_iterative();

  double calibrate();
};

class RobotCameraCalibrator3D {
private:
  RobotCameraCalibrator3DImpl *pImpl;

protected:
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RobotCameraCalibrator3D();
  /**
   * @brief 计算相机到机器人的转换矩阵 (手眼标定)
   * @param _markPoint [in] 标记点的 3D 坐标列表
   * @param _robotPose [in] 机器人位姿列表 (4x4 矩阵)
   * @param type [in] 标定类型: 手在眼上 (EIH) 或 手在眼外 (ETH)
   * @param _Transform [in,out] 相机到机器人的转换矩阵 (4x4)
   * @param _TCP [in,out] 法兰盘或机器人基座坐标系下的标记点坐标
   * @return RMSE 标定误差
   */
  double calibrate(
      std::vector<Eigen::Vector3d> &_markPoint,
      std::vector<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>> &_robotPose,
      CalibrationType3D type, Eigen::Matrix4d &_Transform,
      Eigen::Vector3d &_TCP);

  /**
   * @brief 计算相机到机器人的转换矩阵 (支持 Pose3D 列表和欧拉角转换)
   */
  double calibrate(std::vector<Eigen::Vector3d> &_markPoint,
                   std::vector<Pose3D> &_robotPose, CalibrationType3D type,
                   Eigen::Matrix4d &_Transform, Eigen::Vector3d &_TCP);

  /**
   * @brief 获取标定误差列表
   * @return 包含每组数据误差的向量
   */
  std::vector<double> getCalibrationError() { return calibration_error_; }

  /**
   * @brief 设置机器人位姿的欧拉角类型
   * @param type [in] 旋转顺序类型 (如 XYZ, ZYX 等)
   * @param ref [in] 参考系类型 (内旋/外旋)
   */
  void setRPYType(RPY::RPYType type,
                  RPY::ReferenceType ref = RPY::ReferenceType::EXTRINSIC) {
    rpy_type_ = type;
    ref_type_ = ref;
  }

  virtual ~RobotCameraCalibrator3D();

private:
  std::vector<double> calibration_error_; // 标定误差列表
  RPY::RPYType rpy_type_ = RPY::RPYType::XYZ;
  RPY::ReferenceType ref_type_ = RPY::ReferenceType::EXTRINSIC;

private:
  RobotCameraCalibrator3D(const RobotCameraCalibrator3D &cb) {
    pImpl = nullptr;
  }
  RobotCameraCalibrator3D &operator=(const RobotCameraCalibrator3D &cb) {
    pImpl = nullptr;
    return *this;
  }
};

#endif //  CALIBRATION_ROBOT_CAMERA_3D_H
