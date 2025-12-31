#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief 2D 4-点手眼标定类
 */
class FourPointCalibrator {
public:
  FourPointCalibrator() {}
  virtual ~FourPointCalibrator() {}

  /**
   * @brief 执行 4 点法标定
   * @param image_points [in] 图像上的 4 个像素点
   * @param robot_points [in] 对应的 4 个机器人坐标点 (x, y)
   * @param transform_matrix [out] 输出 2x3 仿射变换矩阵
   * @return 成功返回 true
   */
  bool calibrate(const std::vector<cv::Point2d> &image_points,
                 const std::vector<cv::Point2d> &robot_points,
                 cv::Mat &transform_matrix);

  /**
   * @brief 使用标定结果转换点
   * @param image_point [in] 图像点
   * @param transform_matrix [in] 2x3 仿射变换矩阵
   * @return 转换后的机器人坐标点
   */
  cv::Point2d transform(const cv::Point2d &image_point,
                        const cv::Mat &transform_matrix);
};
