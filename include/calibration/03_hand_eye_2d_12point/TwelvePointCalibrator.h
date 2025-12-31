#ifndef TWELVEPOINTCALIBRATOR_H
#define TWELVEPOINTCALIBRATOR_H

#include "common/Pose2d.h"
#include <opencv2/opencv.hpp>

class TwelvePointCalibrator {
public:
  enum CalibrationType2D {
    EIH = 0,
    ETH
  }; // 2D 标定类型：手在眼上 (EIH) 或 手在眼外 (ETH)
  enum CameraInstallType {
    SameToTCPZ = 0,
    InverseToTCPZ
  }; // 相机安装类型：与 TCP Z 轴同向或反向

  TwelvePointCalibrator() {}
  virtual ~TwelvePointCalibrator() {}

  /**
   * @brief 设置手眼标定类型
   * @param hand_eye_type [in] 手眼标定类型 (EIH 或 ETH)
   */
  void setHandEyeType(CalibrationType2D hand_eye_type) {
    _hand_eye_type = hand_eye_type;
  }

  /**
   * @brief 设置相机安装类型
   * @param install_type [in] 相机安装类型
   */
  void setCameraInstallType(CameraInstallType install_type) {
    _install_type = install_type;
  }

  /**
   * @brief 设置注册点
   * @param registration_point [in] 注册点位姿
   */
  void setRegistrationPoint(Pose2D registration_point) {
    _registration_point = registration_point;
  }

  /**
   * @brief 获取注册点
   * @return 注册点位姿
   */
  Pose2D getRegistrationPoint() { return _registration_point; }

  // 输入机器人位姿
  /**
   * @brief 设置机器人位姿列表
   * @param robot_poses [in] 机器人位姿向量
   */
  void setRobotPoses(std::vector<Pose2D> robot_poses) {
    _robot_poses = robot_poses;
  }

  /**
   * @brief 设置图像标记点列表
   * @param image_mark_points [in] 图像标记点向量
   */
  void setImageMarksPoints(std::vector<Pose2D> image_mark_points) {
    _image_mark_points = image_mark_points;
  }

  /**
   * @brief 获取标定误差
   * @return 误差向量
   */
  std::vector<double> getCalibrationErrors() { return _errors; }

  // 获得转换矩阵
  /**
   * @brief 获取 2x3 转换矩阵
   * @return 2x3 转换矩阵
   */
  Matrix<double, 2, 3> getTransformPose() { return _transform_pose; }

  /**
   * @brief 获取渲染后的旋转中心 (相对于注册点)
   * @return 旋转中心坐标
   */
  Vector2D getRotationCenter() { return _rotation_center; }

  /**
   * @brief 获取原始旋转中心 (像素坐标)
   */
  Vector2D getRawToolCenter() const { return _raw_tool_center; }

  /**
   * @brief 获取拟合半径
   */
  double getFittingRadius() const { return _fitting_radius; }

public:
  /**
   * @brief 执行标定计算
   * @return 计算成功返回 true，否则返回 false
   */
  bool runCalibration();

  /**
   * @brief 2D 坐标转换：图像到机器人
   * @param image_xytheta [in] 图像坐标和角度
   * @param register_point [in] 注册点
   * @param tool_center [in] 工具中心
   * @param trans_pose [in] 转换矩阵
   * @param robot_xyrz [out] 转换后的机器人位姿
   * @return 状态码
   */
  int transformImageToRobotPose(Pose2D image_xytheta, Pose2D register_point,
                                Vector2d tool_center,
                                Matrix<double, 2, 3> trans_pose,
                                Pose2D &robot_xyrz);

private:
  int _estimateAffineTransform(std::vector<cv::Point2d> image_xy,
                               std::vector<cv::Point2d> robot_xy,
                               Matrix<double, 2, 3> &trans_pose);
  std::vector<double> _computeRotationCenter(std::vector<cv::Point2d> image_xy,
                                             Vector2d &tool_center);
  cv::Point2d _rotatePoint(cv::Point2d pic_pix, double angle,
                           Vector2d tool_center, Pose2D register_point);

private:
  CalibrationType2D _hand_eye_type; // 手眼标定类型
  CameraInstallType _install_type;  // 相机安装类型

  Pose2D _registration_point;             // 注册点
  std::vector<Pose2D> _robot_poses;       // 机器人位姿列表
  std::vector<Pose2D> _image_mark_points; // 图像标记点列表

  std::vector<double> _errors;          // 标定误差列表
  Matrix<double, 2, 3> _transform_pose; // 2x3 转换矩阵
  Vector2D _rotation_center;            // 旋转中心 (相对)
  Vector2D _raw_tool_center;            // 旋转中心 (绝对像素)
  double _fitting_radius = 0;           // 拟合半径

  Vector3D _paraMatrix1; // 参数矩阵 1
  Vector3D _paraMatrix2; // 参数矩阵 2
};

#endif // TWELVEPOINTCALIBRATOR_H