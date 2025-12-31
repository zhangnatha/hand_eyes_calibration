#pragma once

#include "common/Math.h"
#include <iostream>

/**
 * @brief 3D 位姿类 (6 自由度)
 * 描述空间位置和姿态。
 * 包含: (x, y, z) 和 欧拉角 (RPY)。
 */
class Pose3D {
public:
  Pose3D() = default;

  Pose3D(double x, double y, double z, double rx, double ry, double rz)
      : _v(Vector3D(x, y, z)), _rpy(RPY(rx, ry, rz)) {}

  Pose3D(const Vector3D &v, const RPY &rpy) : _v(v), _rpy(rpy) {}

  Pose3D(Transform3D tf) : _v(tf.translation()), _rpy(RPY(tf.rotation())) {}

  double &operator[](int i) {
    if (0 <= i && i <= 2) {
      return _v(i);
    } else if (3 <= i && i <= 5) {
      return _rpy[i - 3];
    } else {
      std::cerr << "Index out of range" << std::endl;
    }
    return _v(0);
  }

  const double operator[](int i) const {
    if (0 <= i && i <= 2) {
      return _v(i);
    } else if (3 <= i && i <= 5) {
      return _rpy[i - 3];
    } else {
      std::cerr << "Index out of range" << std::endl;
    }
  }

  // --- 转换与访问器 ---
  /**
   * @brief 获取位置向量
   * @return 3D 位置向量
   */
  Vector3D xyz() { return _v; }

  /**
   * @brief 获取 RPY 欧拉角对象
   * @return RPY 对象
   */
  RPY rpy() { return _rpy; }

  /**
   * @brief 转换为 4x4 齐次变换矩阵
   * @return 齐次变换矩阵
   */
  Transform3D toTransform3D() { return Transform3D(_v, _rpy.toRotation3D()); }

  // 快捷访问器
  /**
   * @brief 获取 x 分量
   * @return x 分量值
   */
  double x() const { return _v[0]; }

  /**
   * @brief 获取 y 分量
   * @return y 分量值
   */
  double y() const { return _v[1]; }

  /**
   * @brief 获取 z 分量
   * @return z 分量值
   */
  double z() const { return _v[2]; }

  /**
   * @brief 获取 Roll 角度
   * @return Roll 角度值
   */
  double rx() const { return _rpy[0]; }

  /**
   * @brief 获取 Pitch 角度
   * @return Pitch 角度值
   */
  double ry() const { return _rpy[1]; }

  /**
   * @brief 获取 Yaw 角度
   * @return Yaw 角度值
   */
  double rz() const { return _rpy[2]; }

private:
  Vector3D _v; // 位置向量
  RPY _rpy;    // 欧拉角
};
