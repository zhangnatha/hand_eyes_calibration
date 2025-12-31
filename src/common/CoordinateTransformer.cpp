#include "common/CoordinateTransformer.h"

cv::Mat CoordinateTransformer::eulerAnglesToRotationMatrix(float rx, float ry,
                                                     float rz) {
  cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(rx), -sin(rx), 0,
                 sin(rx), cos(rx));

  cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(ry), 0, sin(ry), 0, 1, 0,
                 -sin(ry), 0, cos(ry));

  cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(rz), -sin(rz), 0, sin(rz),
                 cos(rz), 0, 0, 0, 1);

  cv::Mat R = R_z * R_y * R_x;
  return R;
}

cv::Vec3f CoordinateTransformer::rotationMatrixToEulerAngles(cv::Mat &R) {
  double zer = 1e-8;
  double rx = 0;
  double ry = 0;
  double rz = 0;

  ry =
      atan2(-R.at<double>(2, 0), sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                                      R.at<double>(1, 0) * R.at<double>(1, 0)));

  if (ry < M_PI / 2 + zer && ry > M_PI / 2 - zer) {
    ry = M_PI / 2;
    rx = atan2(R.at<double>(0, 1), R.at<double>(1, 1));
    rz = 0;
  } else if (ry > -M_PI / 2 - zer && ry < -M_PI / 2 + zer) {
    ry = -M_PI / 2;
    rx = -atan2(R.at<double>(0, 1), R.at<double>(1, 1));
    rz = 0;
  } else {
    rz = atan2(R.at<double>(1, 0) / cos(ry), R.at<double>(0, 0) / cos(ry));
    rx = atan2(R.at<double>(2, 1) / cos(ry), R.at<double>(2, 2) / cos(ry));
  }

  rx = rx / M_PI * 180;
  ry = ry / M_PI * 180;
  rz = rz / M_PI * 180;

  return cv::Vec3d(rx, ry, rz);
}

cv::Mat CoordinateTransformer::toHomogeneousMatrix(Pose3D &pose_3d) {
  double x = pose_3d.x();
  double y = pose_3d.y();
  double z = pose_3d.z();
  double rx = pose_3d.rx();
  double ry = pose_3d.ry();
  double rz = pose_3d.rz();

  cv::Mat R = eulerAnglesToRotationMatrix(rx * CV_PI / 180, ry * CV_PI / 180,
                                          rz * CV_PI / 180);

  cv::Mat T = (cv::Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1),
               R.at<double>(0, 2), x, R.at<double>(1, 0), R.at<double>(1, 1),
               R.at<double>(1, 2), y, R.at<double>(2, 0), R.at<double>(2, 1),
               R.at<double>(2, 2), z, 0, 0, 0, 1);
  return T;
}

Pose3D CoordinateTransformer::fromHomogeneousMatrix(cv::Mat &transform_matrix) {
  if (transform_matrix.rows != 4 || transform_matrix.cols != 4) {
    throw std::invalid_argument("Transformation matrix must be 4x4.");
  }

  cv::Vec3f euler_angles = rotationMatrixToEulerAngles(transform_matrix);

  return Pose3D(transform_matrix.at<double>(0, 3),
                transform_matrix.at<double>(1, 3),
                transform_matrix.at<double>(2, 3), euler_angles[0],
                euler_angles[1], euler_angles[2]);
}
