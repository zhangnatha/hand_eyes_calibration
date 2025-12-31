#include "calibration/03_hand_eye_2d_12point/TwelvePointCalibrator.h"

bool TwelvePointCalibrator::runCalibration() {
  std::vector<cv::Point2d> image_9points;
  std::vector<double> errors;
  double angles[9];
  if (_hand_eye_type == CalibrationType2D::EIH) {
    std::vector<Pose2D> image_marks_c;
    image_marks_c.push_back(_image_mark_points[0]);
    image_marks_c.push_back(_image_mark_points[5]);
    image_marks_c.push_back(_image_mark_points[6]);
    image_marks_c.push_back(_image_mark_points[7]);
    image_marks_c.push_back(_image_mark_points[8]);
    image_marks_c.push_back(_image_mark_points[1]);
    image_marks_c.push_back(_image_mark_points[2]);
    image_marks_c.push_back(_image_mark_points[3]);
    image_marks_c.push_back(_image_mark_points[4]);

    for (int i = 0; i < 9; i++) {
      cv::Point2d point;
      point.x = image_marks_c[i].x();
      point.y = image_marks_c[i].y();
      image_9points.push_back(point);
    }
    angles[0] = _image_mark_points[0].angle();
    angles[1] = _image_mark_points[5].angle();
    angles[2] = _image_mark_points[6].angle();
    angles[3] = _image_mark_points[7].angle();
    angles[4] = _image_mark_points[8].angle();
    angles[5] = _image_mark_points[1].angle();
    angles[6] = _image_mark_points[2].angle();
    angles[7] = _image_mark_points[3].angle();
    angles[8] = _image_mark_points[4].angle();
  } else {
    if (_install_type == CameraInstallType::SameToTCPZ) {
      for (int i = 0; i < 9; i++) {
        cv::Point2d point;
        point.x = _image_mark_points[i].x();
        point.y = _image_mark_points[i].y();
        image_9points.push_back(point);
        angles[i] = _image_mark_points[i].angle();
      }
    } else {
      std::vector<Pose2D> image_marks_c;
      image_marks_c.push_back(_image_mark_points[0]);
      image_marks_c.push_back(_image_mark_points[1]);
      image_marks_c.push_back(_image_mark_points[8]);
      image_marks_c.push_back(_image_mark_points[7]);
      image_marks_c.push_back(_image_mark_points[6]);
      image_marks_c.push_back(_image_mark_points[5]);
      image_marks_c.push_back(_image_mark_points[4]);
      image_marks_c.push_back(_image_mark_points[3]);
      image_marks_c.push_back(_image_mark_points[2]);
      for (int i = 0; i < 9; i++) {
        cv::Point2d point;
        point.x = image_marks_c[i].x();
        point.y = image_marks_c[i].y();
        image_9points.push_back(point);
      }
      angles[0] = _image_mark_points[0].angle();
      angles[1] = _image_mark_points[1].angle();
      angles[2] = _image_mark_points[8].angle();
      angles[3] = _image_mark_points[7].angle();
      angles[4] = _image_mark_points[6].angle();
      angles[5] = _image_mark_points[5].angle();
      angles[6] = _image_mark_points[4].angle();
      angles[7] = _image_mark_points[3].angle();
      angles[8] = _image_mark_points[2].angle();
    }
  }
  std::vector<cv::Point2d> robot_9points;
  for (int i = 0; i < 9; i++) {
    cv::Point2d ro_point;
    ro_point.x = _robot_poses[i].x();
    ro_point.y = _robot_poses[i].y();
    robot_9points.push_back(ro_point);
  }
  Matrix<double, 2, 3> trans_pose;
  int result =
      _estimateAffineTransform(image_9points, robot_9points, trans_pose);
  if (result < 0) {
    return false;
    std::cerr << "[TwelvePointCalibrator] Tansform pose compute error!"
              << std::endl;
  }
  std::vector<cv::Point2d> rotate_p;
  for (int i = 9; i < _robot_poses.size(); i++) {
    cv::Point2d point;
    point.x = _image_mark_points[i].x();
    point.y = _image_mark_points[i].y();
    rotate_p.push_back(point);
  }
  std::vector<double> error_rotate;
  Vector2d rotate_center;
  error_rotate = _computeRotationCenter(rotate_p, rotate_center);
  Vector2d real_center;
  real_center[0] = rotate_center[0] - _registration_point.x();
  real_center[1] = rotate_center[1] - _registration_point.y();
  for (int i = 0; i < 9; i++) {
    Pose2D image_xyz;
    Pose2D robot_xyz;
    Pose2D compute_robot_xyz;

    image_xyz[0] = image_9points[i].x;
    image_xyz[1] = image_9points[i].y;
    image_xyz[2] = angles[i];
    robot_xyz[0] = _robot_poses[i].x();
    robot_xyz[1] = _robot_poses[i].y();
    robot_xyz[2] = _robot_poses[i].angle();
    transformImageToRobotPose(image_xyz, _registration_point, real_center,
                              trans_pose, compute_robot_xyz);
    double one_error = sqrt((robot_xyz[0] - compute_robot_xyz[0]) *
                                (robot_xyz[0] - compute_robot_xyz[0]) +
                            (robot_xyz[1] - compute_robot_xyz[1]) *
                                (robot_xyz[1] - compute_robot_xyz[1]));
    errors.push_back(one_error);
  }
  for (int i = 0; i < error_rotate.size(); i++) {
    double one_error = error_rotate[i];
    errors.push_back(one_error);
  }

  _errors = errors;
  _transform_pose = trans_pose;
  _rotation_center = real_center;

  return true;
}

int TwelvePointCalibrator::_estimateAffineTransform(
    std::vector<cv::Point2d> image_xy, std::vector<cv::Point2d> robot_xy,
    Matrix<double, 2, 3> &trans_pose) {
  if (image_xy.size() < 4 || robot_xy.size() < 4)
    return -1;
  cv::Mat cvEstimateOut(2, 3, CV_8UC1);
#if 1
  cvEstimateOut = cv::estimateAffine2D(image_xy, robot_xy);
#else
  cvEstimateOut = cv::estimateRigidTransform(image_xy, robot_xy, 1);
#endif
  if (cvEstimateOut.empty())
    return -1;
  trans_pose(0, 0) = cvEstimateOut.at<double>(0, 0);
  trans_pose(0, 1) = cvEstimateOut.at<double>(0, 1);
  trans_pose(0, 2) = cvEstimateOut.at<double>(0, 2);
  trans_pose(1, 0) = cvEstimateOut.at<double>(1, 0);
  trans_pose(1, 1) = cvEstimateOut.at<double>(1, 1);
  trans_pose(1, 2) = cvEstimateOut.at<double>(1, 2);

  return 1;
}

std::vector<double>
TwelvePointCalibrator::_computeRotationCenter(std::vector<cv::Point2d> image_xy,
                                              Vector2d &tool_center) {
  typedef double reals;
  const reals Two = 2.0, Three = 3.0, Four = 4.0;

  int i, iter, IterMAX = 99;

  reals Xi, Yi, Zi;
  reals Mz, Mxy, Mxx, Myy, Mxz, Myz, Mzz, Cov_xy, Var_z;
  reals A0, A1, A2, A22, A3, A33;
  reals Dy, xnew, x, ynew, y;
  reals DET, Xcenter, Ycenter;

  cv::Point2d meanXY;
  meanXY.x = 0;
  meanXY.y = 0;
  for (int i = 0; i < image_xy.size(); i++) {
    meanXY.x = meanXY.x + image_xy[i].x;
    meanXY.y = meanXY.y + image_xy[i].y;
  }
  meanXY = meanXY / double(image_xy.size());

  Mxx = Myy = Mxy = Mxz = Myz = Mzz = 0.;

  for (i = 0; i < image_xy.size(); i++) {
    Xi = image_xy[i].x - meanXY.x;
    Yi = image_xy[i].y - meanXY.y;
    Zi = Xi * Xi + Yi * Yi;

    Mxy += Xi * Yi;
    Mxx += Xi * Xi;
    Myy += Yi * Yi;
    Mxz += Xi * Zi;
    Myz += Yi * Zi;
    Mzz += Zi * Zi;
  }
  Mxx /= image_xy.size();
  Myy /= image_xy.size();
  Mxy /= image_xy.size();
  Mxz /= image_xy.size();
  Myz /= image_xy.size();
  Mzz /= image_xy.size();

  Mz = Mxx + Myy;
  Cov_xy = Mxx * Myy - Mxy * Mxy;
  Var_z = Mzz - Mz * Mz;

  A3 = Four * Mz;
  A2 = -Three * Mz * Mz - Mzz;
  A1 = Var_z * Mz + Four * Cov_xy * Mz - Mxz * Mxz - Myz * Myz;
  A0 = Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) -
       Var_z * Cov_xy;

  A22 = A2 + A2;
  A33 = A3 + A3 + A3;

  for (x = 0., y = A0, iter = 0; iter < IterMAX; iter++) {
    Dy = A1 + x * (A22 + A33 * x);

    xnew = x - y / Dy;

    if ((xnew == x) || (!std::isfinite(xnew)))
      break;

    ynew = A0 + xnew * (A1 + xnew * (A2 + xnew * A3));

    if (abs(ynew) >= abs(y))
      break;

    x = xnew;
    y = ynew;
  }

  DET = x * x - x * Mz + Cov_xy;

  Xcenter = (Mxz * (Myy - x) - Myz * Mxy) / DET / Two;
  Ycenter = (Myz * (Mxx - x) - Mxz * Mxy) / DET / Two;

  tool_center[0] = Xcenter + meanXY.x;
  tool_center[1] = Ycenter + meanXY.y;

  reals r = sqrt(Xcenter * Xcenter + Ycenter * Ycenter + Mz);

  reals sum = 0, dx, dy;
  std::vector<double> fiterror;

  for (int i = 0; i < image_xy.size(); i++) {
    dx = image_xy[i].x - tool_center[0];
    dy = image_xy[i].y - tool_center[1];

    double deviation = abs(sqrt(dx * dx + dy * dy) - r);
    fiterror.push_back(deviation);
  }

  _raw_tool_center = tool_center;
  _fitting_radius = r;

  return fiterror;
}

int TwelvePointCalibrator::transformImageToRobotPose(
    Pose2D image_xytheta, Pose2D register_point, Vector2d tool_center,
    Matrix<double, 2, 3> trans_pose, Pose2D &robot_xyrz) {
  double Rz = register_point.angle() - image_xytheta.angle();

  cv::Point2d regp(register_point.x(), register_point.y());
  regp = _rotatePoint(regp, Rz, tool_center, register_point);

  cv::Point3d Robot_regp;
  Robot_regp.x =
      trans_pose(0, 0) * regp.x + trans_pose(0, 1) * regp.y + trans_pose(0, 2);
  Robot_regp.y =
      trans_pose(1, 0) * regp.x + trans_pose(1, 1) * regp.y + trans_pose(1, 2);

  cv::Point3d Robot_imgp;
  Robot_imgp.x = trans_pose(0, 0) * image_xytheta.x() +
                 trans_pose(0, 1) * image_xytheta.y() + trans_pose(0, 2);
  Robot_imgp.y = trans_pose(1, 0) * image_xytheta.x() +
                 trans_pose(1, 1) * image_xytheta.y() + trans_pose(1, 2);

  robot_xyrz[0] = Robot_imgp.x - Robot_regp.x;
  robot_xyrz[1] = Robot_imgp.y - Robot_regp.y;

  robot_xyrz[2] = Rz;
  return 1;
}

cv::Point2d TwelvePointCalibrator::_rotatePoint(cv::Point2d pic_pix,
                                                double angle,
                                                Vector2d tool_center,
                                                Pose2D register_point) {
  double realtoolcx = register_point.x() + tool_center[0];
  double realtoolcy = register_point.y() + tool_center[1];

  double ang = angle / 180 * CV_PI;
  cv::Point2d out;
  out.x = realtoolcx + (pic_pix.x - realtoolcx) * cos(ang) -
          (pic_pix.y - realtoolcy) * sin(ang);
  out.y = realtoolcy + (pic_pix.y - realtoolcy) * cos(ang) +
          (pic_pix.x - realtoolcx) * sin(ang);

  return out;
}