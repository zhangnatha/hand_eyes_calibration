#ifndef ARUCOMARKERDETECTOR_H
#define ARUCOMARKERDETECTOR_H

#include "common/Pose3d.h"
#include <cmath>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#define S_PTR(TYPE)                                                            \
  typedef std::shared_ptr<TYPE> Ptr;                                           \
  typedef std::weak_ptr<TYPE> WPtr;
class ArucoMarkerDetector {
public:
  S_PTR(ArucoMarkerDetector)

  /**
   * @brief 设置输入图像
   * @param input_image [in] 输入的 OpenCV 图像
   */
  void setInputImage(cv::Mat input_image) { this->input_image_ = input_image; }
  /**
   * @brief 获取输入图像
   * @return 输入的 OpenCV 图像
   */
  cv::Mat getInputImage() { return this->input_image_; }

  /**
   * @brief 设置相机内参矩阵
   * @param camera_matric [in] 3x3 相机内参矩阵
   */
  void setCameraMatrix(cv::Mat camera_matric) {
    this->camera_matric_ = camera_matric;
  }
  /**
   * @brief 获取相机内参矩阵
   * @return 3x3 相机内参矩阵
   */
  cv::Mat getCameraMatrix() { return this->camera_matric_; }

  /**
   * @brief 设置畸变系数
   * @param distoration_coeff [in] 1x5 畸变系数矩阵
   */
  void setDistorationCoeff(cv::Mat distoration_coeff) {
    this->distoration_coeff_ = distoration_coeff;
  }
  /**
   * @brief 获取畸变系数
   * @return 1x5 畸变系数矩阵
   */
  cv::Mat getDistorationCoeff() { return this->distoration_coeff_; }

  /**
   * @brief 设置标记边长
   * @param mark_length [in] ArUco 标记的边长 (mm)
   */
  void setMarkLength(double mark_length) { this->mark_length_ = mark_length; }
  /**
   * @brief 获取标记边长
   * @return 标记边长 (mm)
   */
  double getMarkLength() { return this->mark_length_; }

  /**
   * @brief 设置标记位姿列表
   * @param mark_pose [in] 位姿向量
   */
  void setMarkPose(std::vector<Pose3D> mark_pose) {
    this->mark_pose_ = mark_pose;
  }
  /**
   * @brief 获取标记位姿列表
   * @return 位姿向量
   */
  std::vector<Pose3D> getMarkPose() { return this->mark_pose_; }

  /**
   * @brief 设置标记点列表
   * @param mark_points [in] 2D 像素点向量
   */
  void setMarkPoints(std::vector<cv::Point2f> mark_points) {
    this->mark_points_ = mark_points;
  }
  /**
   * @brief 获取标记点列表
   * @return 2D 像素点向量
   */
  std::vector<cv::Point2f> getMarkPoints() { return this->mark_points_; }

  /**
   * @brief 设置绘制图像
   * @param darw_image [in] 用于绘制检测结果的图像
   */
  void setDrawFrame(cv::Mat darw_image) { this->draw_image_ = darw_image; }
  /**
   * @brief 获取绘制图像
   * @return 绘制了检测结果的图像
   */
  cv::Mat getDrawFrame() { return this->draw_image_; }

  /**
   * @brief 执行标记检测和位姿估计
   * @return 计算成功返回 true，否则返回 false
   */
  bool detectMarkersAndEstimatePose();

private:
  std::vector<cv::Point3f> _generate4MarkerWorldCoords(double mark_length);
  std::vector<cv::Point3f> _generate9MarkerWorldCoords(double mark_length);
  std::vector<cv::Point2f>
  _extractMarkerCorners(std::vector<std::vector<cv::Point2f>> corners);
  Pose3D _convertOpenCVPoseToPose3D(cv::Vec3d rvec, cv::Vec3d tvec);

  std::vector<Pose3D> mark_pose_;        // 标记位姿列表
  cv::Mat input_image_;                  // 输入图像
  cv::Mat camera_matric_;                // 相机内参矩阵
  cv::Mat distoration_coeff_;            // 畸变系数
  double mark_length_;                   // 标记边长
  std::vector<cv::Point2f> mark_points_; // 标记点列表
  cv::Mat draw_image_;                   // 绘制图像
};

#endif // ARUCOMARKERDETECTOR_H
