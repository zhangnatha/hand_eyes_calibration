#include "calibration/02_hand_eye_2d_4point/FourPointCalibrator.h"

bool FourPointCalibrator::calibrate(const std::vector<cv::Point2d> &image_points,
                                   const std::vector<cv::Point2d> &robot_points,
                                   cv::Mat &transform_matrix) {
  if (image_points.size() < 4 || robot_points.size() < 4) {
    return false;
  }
  transform_matrix = cv::estimateAffine2D(image_points, robot_points);
  return !transform_matrix.empty();
}

cv::Point2d FourPointCalibrator::transform(const cv::Point2d &image_point,
                                          const cv::Mat &transform_matrix) {
  if (transform_matrix.empty()) {
    return cv::Point2d(0, 0);
  }
  cv::Point2d robot_point;
  double a = transform_matrix.at<double>(0, 0);
  double b = transform_matrix.at<double>(0, 1);
  double c = transform_matrix.at<double>(0, 2);
  double d = transform_matrix.at<double>(1, 0);
  double e = transform_matrix.at<double>(1, 1);
  double f = transform_matrix.at<double>(1, 2);
  robot_point.x = a * image_point.x + b * image_point.y + c;
  robot_point.y = d * image_point.x + e * image_point.y + f;
  return robot_point;
}
