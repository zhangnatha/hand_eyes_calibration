#pragma once

#include "common/Pose3d.h"
#include <opencv2/opencv.hpp>

/**
 * @brief 位姿变换工具类
 * 提供欧拉角、矩阵和 Pose3D 之间的静态转换函数。
 */
class CoordinateTransformer {
public:
  /**
   * @brief 欧拉角转旋转矩阵 (Z-Y-X 顺序)
   * @param rx [in] 绕 X 轴旋转角度 (弧度)
   * @param ry [in] 绕 Y 轴旋转角度 (弧度)
   * @param rz [in] 绕 Z 轴旋转角度 (弧度)
   * @return 3x3 旋转矩阵
   */
  static cv::Mat eulerAnglesToRotationMatrix(float rx, float ry, float rz);

  /**
   * @brief 旋转矩阵转欧拉角 (Z-Y-X 顺序)
   * @param R [in] 3x3 旋转矩阵
   * @return 包含 rx, ry, rz 的向量 (角度)
   */
  static cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);

  /**
   * @brief Pose3D 转 4x4 齐次变换矩阵
   * @param pose_3d [in] Pose3D 对象
   * @return 4x4 齐次变换矩阵
   */
  static cv::Mat toHomogeneousMatrix(Pose3D &pose_3d);

  /**
   * @brief 4x4 齐次变换矩阵转 Pose3D
   * @param transform_matrix [in] 4x4 齐次变换矩阵
   * @return Pose3D 对象
   */
  static Pose3D fromHomogeneousMatrix(cv::Mat &transform_matrix);
};
