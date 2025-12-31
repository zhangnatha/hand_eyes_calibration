#pragma once

#include "Math.h"

/**
 * @brief 2D 位姿类 (3 自由度)
 * 描述平面机器人状态。
 * 包含: (x, y) 和 角度。
 */
class Pose2D {
public:
  Pose2D() : _v(Vector2D(0, 0)), _angle(0) {}

  Pose2D(double x, double y, double angle)
      : _v(Vector2D(x, y)), _angle(angle) {}

  Pose2D(const Vector2D &v, double angle) : _v(v), _angle(angle) {}

  double &operator[](int i) {
    if (0 <= i && i <= 1) {
      return _v(i);
    } else if (i == 2) {
      return _angle;
    } else {
      std::cerr << "unexpected behavior!" << std::endl;
      return _v(0);
    }
  }

  /**
   * @brief 下标访问 (只读)
   * @param i 索引: 0->x, 1->y, 2->angle
   */
  const double operator[](int i) const {
    if (0 <= i && i <= 1) {
      double value = _v(i);
      return value;
    } else if (i == 2) {
      double value = _angle;
      return value;
    } else {
      std::cerr << "unexpected behavior!" << std::endl;
    }
  }

  // --- 访问器 ---
  /**
   * @brief 获取 x 分量
   * @return x 分量值
   */
  const double x() const {
    double value = _v(0);
    return value;
  }

  /**
   * @brief 获取 y 分量
   * @return y 分量值
   */
  const double y() const {
    double value = _v(1);
    return value;
  }

  /**
   * @brief 获取角度
   * @return 角度值
   */
  const double angle() const { return _angle; }

  /**
   * @brief 获取中心位置向量
   * @return 2D 位置向量
   */
  Vector2D center() { return _v; }

public:
  Vector2D _v;   // 位置 (x, y)
  double _angle; // 角度
};
