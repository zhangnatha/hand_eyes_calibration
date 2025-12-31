#ifndef CHESSBOARD_CALIBRATOR_H
#define CHESSBOARD_CALIBRATOR_H

#include "common/CalibrationTypes.h"
#include "common/Pose3d.h"
#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief 标定板检测与PnP解算类
 * 负责识别图像中的标定板角点，并计算相机相对于标定板的位姿。
 */
class ChessBoardCalibrator {
public:
  ChessBoardCalibrator() {}
  virtual ~ChessBoardCalibrator() {}

  /**
   * @brief 生成标定板的世界坐标系角点 (假设 Z=0)
   * @param world_corners [out] 输出的 3D 点集
   */
  void worldCoordinate(std::vector<cv::Point3f> &world_corners);

  /**
   * @brief 检测圆点阵列标定板的角点
   * @param input_image [in] 输入图像
   * @param output_corners_image [out] 输出带有角点标记的图像
   * @param pixel_corners [out] 输出检测到的 2D 像素坐标
   * @param world_corners [out] 输出对应的 3D 世界坐标
   * @param circle_area [in] 圆斑面积过滤参数
   * @return 0 成功, 1 失败
   */
  int finderCircleCorners(cv::Mat input_image, cv::Mat &output_corners_image,
                          std::vector<cv::Point2f> &pixel_corners,
                          std::vector<cv::Point3f> &world_corners,
                          int circle_area);

  /**
   * @brief PnP 位姿解算
   * @param world_corners [in] 3D 世界坐标点集
   * @param pixel_corners [in] 2D 像素坐标点集
   * @param camera_matrix [in] 相机内参矩阵
   * @param dist_coeffs [in] 畸变系数
   * @param pose [out] 解算出的位姿 (Pose3D)
   * @return 解算成功返回 true，否则返回 false
   */
  bool solverPnP(std::vector<cv::Point3f> world_corners,
                 std::vector<cv::Point2f> pixel_corners, cv::Mat camera_matrix,
                 cv::Mat dist_coeffs, Pose3D &pose);

  /**
   * @brief 设置标定板尺寸和间距
   * @param cols [in] 列数
   * @param rows [in] 行数
   * @param interval [in] 间距 (mm)
   */
  void setBoardSize(int cols, int rows, float interval) {
    board_size_cols_ = cols;
    board_size_rows_ = rows;
    board_size_interval_ = interval;
  }
  /**
   * @brief 获取标定板尺寸和间距
   * @param cols [out] 列数
   * @param rows [out] 行数
   * @param interval [out] 间距 (mm)
   */
  void getBoardSize(int &cols, int &rows, float &interval) {
    cols = board_size_cols_;
    rows = board_size_rows_;
    interval = board_size_interval_;
  }

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
  cv::Mat getCameraMatrix() { return camera_matric_; }

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
  cv::Mat getDistorationCoeff() { return distoration_coeff_; }

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
  std::vector<Pose3D> getMarkPose() { return mark_pose_; }

  /**
   * @brief 计算位姿
   * @return 计算成功返回 true，否则返回 false
   */
  bool computePose();

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
  std::vector<cv::Point2f> getMarkPoints() { return mark_points_; }

  /**
   * @brief 设置世界坐标点列表
   * @param world_points [in] 3D 世界坐标点向量
   */
  void setWorldPoints(std::vector<cv::Point3f> world_points) {
    this->world_points_ = world_points;
  }
  /**
   * @brief 获取世界坐标点列表
   * @return 3D 世界坐标点向量
   */
  std::vector<cv::Point3f> getWorldPoints() { return world_points_; }

private:
  int board_size_cols_, board_size_rows_;     // 标定板行列数
  float board_size_interval_;                 // 标定板点间距
  cv::Mat camera_matric_, distoration_coeff_; // 相机内参和畸变系数
  std::vector<Pose3D> mark_pose_;             // 标记位姿列表
  std::vector<cv::Point2f> mark_points_;      // 标记点列表
  std::vector<cv::Point3f> world_points_;     // 世界坐标点列表
};

#endif // CHESSBOARD_CALIBRATOR_H
