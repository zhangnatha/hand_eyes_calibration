#include "common/ArucoMarkerDetector.h"

std::vector<cv::Point3f>
ArucoMarkerDetector::_generate4MarkerWorldCoords(double mark_length) {
  double light_line = mark_length / 13;
  auto intervalLength = (mark_length - light_line) / 2;
  std::vector<cv::Point3f> objPoints;
  objPoints.push_back(cv::Point3f(0.f, 0.f, 0.f));
  objPoints.push_back(cv::Point3f(intervalLength, 0.f, 0.f));
  objPoints.push_back(cv::Point3f(intervalLength, intervalLength, 0.f));
  objPoints.push_back(cv::Point3f(0.f, intervalLength, 0.f));

  objPoints.push_back(cv::Point3f(intervalLength + light_line, 0.f, 0.f));
  objPoints.push_back(cv::Point3f(mark_length, 0.f, 0.f));
  objPoints.push_back(cv::Point3f(mark_length, intervalLength, 0.f));
  objPoints.push_back(
      cv::Point3f(intervalLength + light_line, intervalLength, 0.f));

  objPoints.push_back(cv::Point3f(0.f, intervalLength + light_line, 0.f));
  objPoints.push_back(
      cv::Point3f(intervalLength, intervalLength + light_line, 0.f));
  objPoints.push_back(cv::Point3f(intervalLength, mark_length, 0.f));
  objPoints.push_back(cv::Point3f(0.f, mark_length, 0.f));

  objPoints.push_back(cv::Point3f(intervalLength + light_line,
                                  intervalLength + light_line, 0.f));
  objPoints.push_back(
      cv::Point3f(mark_length, intervalLength + light_line, 0.f));
  objPoints.push_back(cv::Point3f(mark_length, mark_length, 0.f));
  objPoints.push_back(
      cv::Point3f(intervalLength + light_line, mark_length, 0.f));
  return objPoints;
}

std::vector<cv::Point3f>
ArucoMarkerDetector::_generate9MarkerWorldCoords(double mark_length) {
  double light_line = mark_length / 20;
  auto intervalLength = (mark_length - 2 * light_line) / 3;
  std::vector<cv::Point3f> objPoints;
  objPoints.push_back(cv::Point3f(0.f, 0.f, 0.f));
  objPoints.push_back(cv::Point3f(intervalLength, 0.f, 0.f));
  objPoints.push_back(cv::Point3f(intervalLength, intervalLength, 0.f));
  objPoints.push_back(cv::Point3f(0.f, intervalLength, 0.f));

  objPoints.push_back(cv::Point3f(intervalLength + light_line, 0.f, 0.f));
  objPoints.push_back(cv::Point3f(2 * intervalLength + light_line, 0.f, 0.f));
  objPoints.push_back(
      cv::Point3f(2 * intervalLength + light_line, intervalLength, 0.f));
  objPoints.push_back(
      cv::Point3f(intervalLength + light_line, intervalLength, 0.f));

  objPoints.push_back(
      cv::Point3f(2 * intervalLength + 2 * light_line, 0.f, 0.f));
  objPoints.push_back(cv::Point3f(mark_length, 0.f, 0.f));
  objPoints.push_back(cv::Point3f(mark_length, intervalLength, 0.f));
  objPoints.push_back(
      cv::Point3f(2 * intervalLength + 2 * light_line, intervalLength, 0.f));

  objPoints.push_back(cv::Point3f(0.f, intervalLength + light_line, 0.f));
  objPoints.push_back(
      cv::Point3f(intervalLength, intervalLength + light_line, 0.f));
  objPoints.push_back(
      cv::Point3f(intervalLength, 2 * intervalLength + light_line, 0.f));
  objPoints.push_back(cv::Point3f(0.f, 2 * intervalLength + light_line, 0.f));

  objPoints.push_back(cv::Point3f(intervalLength + light_line,
                                  intervalLength + light_line, 0.f));
  objPoints.push_back(cv::Point3f(2 * intervalLength + light_line,
                                  intervalLength + light_line, 0.f));
  objPoints.push_back(cv::Point3f(2 * intervalLength + light_line,
                                  2 * intervalLength + light_line, 0.f));
  objPoints.push_back(cv::Point3f(intervalLength + light_line,
                                  2 * intervalLength + light_line, 0.f));

  objPoints.push_back(cv::Point3f(2 * intervalLength + 2 * light_line,
                                  intervalLength + light_line, 0.f));
  objPoints.push_back(
      cv::Point3f(mark_length, intervalLength + light_line, 0.f));
  objPoints.push_back(
      cv::Point3f(mark_length, 2 * intervalLength + light_line, 0.f));
  objPoints.push_back(cv::Point3f(2 * intervalLength + 2 * light_line,
                                  2 * intervalLength + light_line, 0.f));

  objPoints.push_back(
      cv::Point3f(0.f, 2 * intervalLength + 2 * light_line, 0.f));
  objPoints.push_back(
      cv::Point3f(intervalLength, 2 * intervalLength + 2 * light_line, 0.f));
  objPoints.push_back(cv::Point3f(intervalLength, mark_length, 0.f));
  objPoints.push_back(cv::Point3f(0.f, mark_length, 0.f));

  objPoints.push_back(cv::Point3f(intervalLength + light_line,
                                  2 * intervalLength + 2 * light_line, 0.f));
  objPoints.push_back(cv::Point3f(2 * intervalLength + light_line,
                                  2 * intervalLength + 2 * light_line, 0.f));
  objPoints.push_back(
      cv::Point3f(2 * intervalLength + light_line, mark_length, 0.f));
  objPoints.push_back(
      cv::Point3f(intervalLength + light_line, mark_length, 0.f));

  objPoints.push_back(cv::Point3f(2 * intervalLength + 2 * light_line,
                                  2 * intervalLength + 2 * light_line, 0.f));
  objPoints.push_back(
      cv::Point3f(mark_length, 2 * intervalLength + 2 * light_line, 0.f));
  objPoints.push_back(cv::Point3f(mark_length, mark_length, 0.f));
  objPoints.push_back(
      cv::Point3f(2 * intervalLength + 2 * light_line, mark_length, 0.f));
  return objPoints;
}

std::vector<cv::Point2f> ArucoMarkerDetector::_extractMarkerCorners(
    std::vector<std::vector<cv::Point2f>> corners) {
  std::vector<cv::Point2f> corner;
  for (int i = 0; i < corners.size(); i++) {
    for (int j = 0; j < corners[i].size(); j++) {
      corner.push_back(cv::Point2f(corners[i][j].x, corners[i][j].y));
    }
  }
  return corner;
}

Pose3D ArucoMarkerDetector::_convertOpenCVPoseToPose3D(cv::Vec3d rvec,
                                                     cv::Vec3d tvec) {
  cv::Mat temp_Hextr(4, 4, CV_64FC1);
  cv::Mat R(3, 3, CV_64FC1);
  cv::Mat Rv(1, 3, CV_64FC1);

  Rv.at<double>(0, 0) = rvec[0];
  Rv.at<double>(0, 1) = rvec[1];
  Rv.at<double>(0, 2) = rvec[2];
  cv::Rodrigues(Rv, R);
  R.copyTo(temp_Hextr(cv::Rect(0, 0, 3, 3)));
  temp_Hextr.at<double>(0, 3) = tvec[0];
  temp_Hextr.at<double>(1, 3) = tvec[1];
  temp_Hextr.at<double>(2, 3) = tvec[2];
  temp_Hextr.at<double>(3, 0) = 0.0;
  temp_Hextr.at<double>(3, 1) = 0.0;
  temp_Hextr.at<double>(3, 2) = 0.0;
  temp_Hextr.at<double>(3, 3) = 1.0;
  Transform3D tf_pose;
  tf_pose(0, 0) = temp_Hextr.at<double>(0, 0);
  tf_pose(0, 1) = temp_Hextr.at<double>(0, 1);
  tf_pose(0, 2) = temp_Hextr.at<double>(0, 2);
  tf_pose(0, 3) = temp_Hextr.at<double>(0, 3);
  tf_pose(1, 0) = temp_Hextr.at<double>(1, 0);
  tf_pose(1, 1) = temp_Hextr.at<double>(1, 1);
  tf_pose(1, 2) = temp_Hextr.at<double>(1, 2);
  tf_pose(1, 3) = temp_Hextr.at<double>(1, 3);
  tf_pose(2, 0) = temp_Hextr.at<double>(2, 0);
  tf_pose(2, 1) = temp_Hextr.at<double>(2, 1);
  tf_pose(2, 2) = temp_Hextr.at<double>(2, 2);
  tf_pose(2, 3) = temp_Hextr.at<double>(2, 3);
  tf_pose(3, 0) = temp_Hextr.at<double>(3, 0);
  tf_pose(3, 1) = temp_Hextr.at<double>(3, 1);
  tf_pose(3, 2) = temp_Hextr.at<double>(3, 2);
  tf_pose(3, 3) = temp_Hextr.at<double>(3, 3);
  Pose3D pose_tf(tf_pose);
  return pose_tf;
}

bool ArucoMarkerDetector::detectMarkersAndEstimatePose() {
  cv::Mat input_image = getInputImage();
  double mark_length = getMarkLength();
  cv::Mat camera_matrix = getCameraMatrix();
  cv::Mat distoration_coeff = getDistorationCoeff();
  std::vector<Pose3D> mark_pose;

  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  std::vector<std::vector<cv::Point2f>> corners, rejectedImgPoints;
  std::vector<int> ids;
  auto parameters = cv::aruco::DetectorParameters::create();
  parameters->cornerRefinementMethod =
      cv::aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX;
  parameters->cornerRefinementWinSize = 7;
  parameters->cornerRefinementMaxIterations = 100;

  cv::aruco::detectMarkers(input_image, dictionary, corners, ids, parameters,
                           rejectedImgPoints);
  if (ids.size() == 9) {
    cv::Mat draw_image;
    if (input_image.channels() != 3)
      cvtColor(input_image, draw_image, cv::COLOR_GRAY2RGB);
    else {
      draw_image = input_image;
    }
    std::vector<std::vector<cv::Point2f>> corners_order;
    corners_order.resize(corners.size());
    int mark_num0 = 0;
    int mark_num1 = 0;
    int mark_num2 = 0;
    int mark_num3 = 0;
    int mark_num4 = 0;
    int mark_num5 = 0;
    int mark_num6 = 0;
    int mark_num7 = 0;
    int mark_num8 = 0;

    for (int i = 0; i < 9; i++) {
      if (ids[i] == 0) {
        corners_order[0] = corners[i];
        mark_num0 = mark_num0 + 1;
      } else if (ids[i] == 1) {
        corners_order[1] = corners[i];
        mark_num1 = mark_num1 + 1;
      } else if (ids[i] == 2) {
        corners_order[2] = corners[i];
        mark_num2 = mark_num2 + 1;
      } else if (ids[i] == 3) {
        corners_order[3] = corners[i];
        mark_num3 = mark_num3 + 1;
      } else if (ids[i] == 4) {
        corners_order[4] = corners[i];
        mark_num3 = mark_num3 + 1;
      } else if (ids[i] == 5) {
        corners_order[5] = corners[i];
        mark_num3 = mark_num3 + 1;
      } else if (ids[i] == 6) {
        corners_order[6] = corners[i];
        mark_num3 = mark_num3 + 1;
      } else if (ids[i] == 7) {
        corners_order[7] = corners[i];
        mark_num3 = mark_num3 + 1;
      } else {
        corners_order[8] = corners[i];
        mark_num3 = mark_num3 + 1;
      }
    }
    if (mark_num0 == mark_num1 == mark_num2 == mark_num3 == mark_num4 ==
        mark_num5 == mark_num6 == mark_num7 == mark_num8 == 1) {
      cv::Vec3d rvec;
      cv::Vec3d tvec;
      std::vector<cv::Point2f> corner = _extractMarkerCorners(corners_order);
      std::vector<cv::Point3f> objPoints =
          _generate9MarkerWorldCoords(mark_length);
      auto compute = solvePnP(objPoints, corner, camera_matrix,
                              distoration_coeff, rvec, tvec);
      if (compute == false) {
        return false;
      }
      Pose3D pose_out = _convertOpenCVPoseToPose3D(rvec, tvec);
      mark_pose.push_back(pose_out);
      float length = mark_length / 4;
      int thickness = 2;
      cv::drawFrameAxes(draw_image, camera_matrix, distoration_coeff, rvec,
                        tvec, length, thickness);
      setDrawFrame(draw_image);
      setMarkPoints(corner);
      setMarkPose(mark_pose);
      return true;
    } else {
      return false;
    }
  } else if (ids.size() == 4) {
    cv::Mat draw_image;
    if (input_image.channels() != 3)
      cvtColor(input_image, draw_image, cv::COLOR_GRAY2RGB);
    else {
      draw_image = input_image;
    }
    std::vector<std::vector<cv::Point2f>> corners_order;
    corners_order.resize(corners.size());
    int mark_num0 = 0;
    int mark_num1 = 0;
    int mark_num2 = 0;
    int mark_num3 = 0;
    for (int i = 0; i < 4; i++) {
      if (ids[i] == 0) {
        corners_order[0] = corners[i];
        mark_num0 = mark_num0 + 1;
      } else if (ids[i] == 1) {
        corners_order[1] = corners[i];
        mark_num1 = mark_num1 + 1;
      } else if (ids[i] == 2) {
        corners_order[2] = corners[i];
        mark_num2 = mark_num2 + 1;
      } else {
        corners_order[3] = corners[i];
        mark_num3 = mark_num3 + 1;
      }
    }
    if (mark_num0 == mark_num1 == mark_num2 == mark_num3 == 1) {
      cv::Vec3d rvec;
      cv::Vec3d tvec;
      std::vector<cv::Point2f> corner = _extractMarkerCorners(corners_order);
      std::vector<cv::Point3f> objPoints =
          _generate4MarkerWorldCoords(mark_length);
      auto compute = cv::solvePnP(objPoints, corner, camera_matrix,
                                  distoration_coeff, rvec, tvec);
      if (compute == false) {
        return false;
      }
      Pose3D pose_out = _convertOpenCVPoseToPose3D(rvec, tvec);
      mark_pose.push_back(pose_out);
      float length = mark_length / 4;
      int thickness = 2;
      cv::drawFrameAxes(draw_image, camera_matrix, distoration_coeff, rvec,
                        tvec, length, thickness);
      setDrawFrame(draw_image);
      setMarkPoints(corner);
      setMarkPose(mark_pose);
      return true;
    } else {
      return false;
    }
  } else if (ids.size() > 1) {
    cv::Mat draw_image;
    if (input_image.channels() != 3)
      cvtColor(input_image, draw_image, cv::COLOR_GRAY2RGB);
    else {
      draw_image = input_image;
    }
    std::vector<std::vector<cv::Point2f>> corners_order;
    corners_order.resize(4);
    if (ids[0] == 0) {
      corners_order[0] = corners[0];
      std::vector<cv::Point2f> corners1;
      corners1.resize(4);
      cv::Point2f axis012 = cv::Point2f(corners[0][1].x - corners[0][0].x,
                                        corners[0][1].y - corners[0][0].y);
      cv::Point2f axis043 = cv::Point2f(corners[0][2].x - corners[0][3].x,
                                        corners[0][2].y - corners[0][3].y);
      cv::Point2f axis11 = cv::Point2f(corners[0][0].x + (7 / 6) * axis012.x,
                                       corners[0][0].y + (7 / 6) * axis012.y);
      cv::Point2f axis12 = cv::Point2f(corners[0][0].x + (13 / 6) * axis012.x,
                                       corners[0][0].y + (13 / 6) * axis012.y);
      cv::Point2f axis13 = cv::Point2f(corners[0][3].x + (7 / 6) * axis043.x,
                                       corners[0][3].y + (7 / 6) * axis043.y);
      cv::Point2f axis14 = cv::Point2f(corners[0][3].x + (13 / 6) * axis043.x,
                                       corners[0][3].y + (13 / 6) * axis043.y);
      corners1[0] = axis11;
      corners1[1] = axis12;
      corners1[2] = axis13;
      corners1[3] = axis14;
      corners_order[1] = corners1;

      std::vector<cv::Point2f> corners2;
      corners2.resize(4);
      cv::Point2f axis014 = cv::Point2f(corners[0][3].x - corners[0][0].x,
                                        corners[0][3].y - corners[0][0].y);
      cv::Point2f axis023 = cv::Point2f(corners[0][2].x - corners[0][1].x,
                                        corners[0][2].y - corners[0][1].y);
      cv::Point2f axis21 = cv::Point2f(corners[0][0].x + (7 / 6) * axis014.x,
                                       corners[0][0].y + (7 / 6) * axis014.y);
      cv::Point2f axis22 = cv::Point2f(corners[0][1].x + (7 / 6) * axis023.x,
                                       corners[0][1].y + (7 / 6) * axis023.y);
      cv::Point2f axis23 = cv::Point2f(corners[0][1].x + (13 / 6) * axis023.x,
                                       corners[0][1].y + (13 / 6) * axis023.y);
      cv::Point2f axis24 = cv::Point2f(corners[0][0].x + (13 / 6) * axis014.x,
                                       corners[0][0].y + (13 / 6) * axis014.y);
      corners2[0] = axis21;
      corners2[1] = axis22;
      corners2[2] = axis23;
      corners2[3] = axis24;
      corners_order[2] = corners2;

      std::vector<cv::Point2f> corners3;
      corners3.resize(4);
      cv::Point2f axis114 = cv::Point2f(corners1[3].x - corners1[0].x,
                                        corners1[3].y - corners1[0].y);
      cv::Point2f axis123 = cv::Point2f(corners1[2].x - corners1[1].x,
                                        corners1[2].y - corners1[1].y);
      cv::Point2f axis31 = cv::Point2f(corners1[0].x + (7 / 6) * axis114.x,
                                       corners1[0].y + (7 / 6) * axis114.y);
      cv::Point2f axis32 = cv::Point2f(corners1[1].x + (7 / 6) * axis123.x,
                                       corners1[1].y + (7 / 6) * axis123.y);
      cv::Point2f axis33 = cv::Point2f(corners1[1].x + (13 / 6) * axis123.x,
                                       corners1[1].y + (13 / 6) * axis123.y);
      cv::Point2f axis34 = cv::Point2f(corners1[0].x + (13 / 6) * axis114.x,
                                       corners1[0].y + (13 / 6) * axis114.y);
      corners3[0] = axis31;
      corners3[1] = axis32;
      corners3[2] = axis33;
      corners3[3] = axis34;
      corners_order[3] = corners3;
    } else if (ids[0] == 1) {
      std::vector<cv::Point2f> corners0;
      corners0.resize(4);
      cv::Point2f axis121 = cv::Point2f(corners[0][0].x - corners[0][1].x,
                                        corners[0][0].y - corners[0][1].y);
      cv::Point2f axis134 = cv::Point2f(corners[0][3].x - corners[0][2].x,
                                        corners[0][3].y - corners[0][2].y);
      cv::Point2f axis01 = cv::Point2f(corners[0][1].x + (13 / 6) * axis121.x,
                                       corners[0][1].y + (13 / 6) * axis121.y);
      cv::Point2f axis02 = cv::Point2f(corners[0][1].x + (7 / 6) * axis121.x,
                                       corners[0][1].y + (7 / 6) * axis121.y);
      cv::Point2f axis03 = cv::Point2f(corners[0][2].x + (7 / 6) * axis134.x,
                                       corners[0][2].y + (7 / 6) * axis134.y);
      cv::Point2f axis04 = cv::Point2f(corners[0][2].x + (13 / 6) * axis134.x,
                                       corners[0][2].y + (13 / 6) * axis134.y);
      corners0[0] = axis01;
      corners0[1] = axis02;
      corners0[2] = axis03;
      corners0[3] = axis04;
      corners_order[0] = corners0;

      corners_order[1] = corners[0];

      std::vector<cv::Point2f> corners2;
      corners2.resize(4);
      cv::Point2f axis014 = cv::Point2f(corners0[3].x - corners0[0].x,
                                        corners0[3].y - corners0[0].y);
      cv::Point2f axis023 = cv::Point2f(corners0[2].x - corners0[1].x,
                                        corners0[2].y - corners0[1].y);
      cv::Point2f axis21 = cv::Point2f(corners0[0].x + (7 / 6) * axis014.x,
                                       corners0[0].y + (7 / 6) * axis014.y);
      cv::Point2f axis22 = cv::Point2f(corners0[1].x + (7 / 6) * axis023.x,
                                       corners0[1].y + (7 / 6) * axis023.y);
      cv::Point2f axis23 = cv::Point2f(corners0[1].x + (13 / 6) * axis023.x,
                                       corners0[1].y + (13 / 6) * axis023.y);
      cv::Point2f axis24 = cv::Point2f(corners0[0].x + (13 / 6) * axis014.x,
                                       corners0[0].y + (13 / 6) * axis014.y);
      corners2[0] = axis21;
      corners2[1] = axis22;
      corners2[2] = axis23;
      corners2[3] = axis24;
      corners_order[2] = corners2;

      std::vector<cv::Point2f> corners3;
      corners3.resize(4);
      cv::Point2f axis114 = cv::Point2f(corners[0][3].x - corners[0][0].x,
                                        corners[0][3].y - corners[0][0].y);
      cv::Point2f axis123 = cv::Point2f(corners[0][2].x - corners[0][1].x,
                                        corners[0][2].y - corners[0][1].y);
      cv::Point2f axis31 = cv::Point2f(corners[0][0].x + (7 / 6) * axis114.x,
                                       corners[0][0].y + (7 / 6) * axis114.y);
      cv::Point2f axis32 = cv::Point2f(corners[0][1].x + (7 / 6) * axis123.x,
                                       corners[0][1].y + (7 / 6) * axis123.y);
      cv::Point2f axis33 = cv::Point2f(corners[0][1].x + (13 / 6) * axis123.x,
                                       corners[0][1].y + (13 / 6) * axis123.y);
      cv::Point2f axis34 = cv::Point2f(corners[0][0].x + (13 / 6) * axis114.x,
                                       corners[0][0].y + (13 / 6) * axis114.y);
      corners3[0] = axis31;
      corners3[1] = axis32;
      corners3[2] = axis33;
      corners3[3] = axis34;
      corners_order[3] = corners3;
    } else if (ids[0] == 2) {
      std::vector<cv::Point2f> corners0;
      corners0.resize(4);
      cv::Point2f axis241 = cv::Point2f(corners[0][0].x - corners[0][3].x,
                                        corners[0][0].y - corners[0][3].y);
      cv::Point2f axis232 = cv::Point2f(corners[0][1].x - corners[0][2].x,
                                        corners[0][1].y - corners[0][2].y);
      cv::Point2f axis01 = cv::Point2f(corners[0][3].x + (13 / 6) * axis241.x,
                                       corners[0][3].y + (13 / 6) * axis241.y);
      cv::Point2f axis02 = cv::Point2f(corners[0][2].x + (13 / 6) * axis232.x,
                                       corners[0][2].y + (13 / 6) * axis232.y);
      cv::Point2f axis03 = cv::Point2f(corners[0][2].x + (7 / 6) * axis232.x,
                                       corners[0][2].y + (7 / 6) * axis232.y);
      cv::Point2f axis04 = cv::Point2f(corners[0][3].x + (7 / 6) * axis241.x,
                                       corners[0][3].y + (7 / 6) * axis241.y);
      corners0[0] = axis01;
      corners0[1] = axis02;
      corners0[2] = axis03;
      corners0[3] = axis04;
      corners_order[0] = corners0;

      std::vector<cv::Point2f> corners1;
      corners1.resize(4);
      cv::Point2f axis012 = cv::Point2f(corners0[1].x - corners0[0].x,
                                        corners0[1].y - corners0[0].y);
      cv::Point2f axis043 = cv::Point2f(corners0[2].x - corners0[3].x,
                                        corners0[2].y - corners0[3].y);
      cv::Point2f axis11 = cv::Point2f(corners0[0].x + (7 / 6) * axis012.x,
                                       corners0[0].y + (7 / 6) * axis012.y);
      cv::Point2f axis12 = cv::Point2f(corners0[0].x + (13 / 6) * axis012.x,
                                       corners0[0].y + (13 / 6) * axis012.y);
      cv::Point2f axis13 = cv::Point2f(corners0[3].x + (13 / 6) * axis043.x,
                                       corners0[3].y + (13 / 6) * axis043.y);
      cv::Point2f axis14 = cv::Point2f(corners0[3].x + (7 / 6) * axis043.x,
                                       corners0[3].y + (7 / 6) * axis043.y);
      corners1[0] = axis11;
      corners1[1] = axis12;
      corners1[2] = axis13;
      corners1[3] = axis14;
      corners_order[1] = corners1;

      corners_order[2] = corners[0];

      std::vector<cv::Point2f> corners3;
      corners3.resize(4);
      cv::Point2f axis212 = cv::Point2f(corners[0][1].x - corners[0][0].x,
                                        corners[0][1].y - corners[0][0].y);
      cv::Point2f axis243 = cv::Point2f(corners[0][2].x - corners[0][3].x,
                                        corners[0][2].y - corners[0][3].y);
      cv::Point2f axis31 = cv::Point2f(corners[0][0].x + (7 / 6) * axis212.x,
                                       corners[0][0].y + (7 / 6) * axis212.y);
      cv::Point2f axis32 = cv::Point2f(corners[0][0].x + (13 / 6) * axis212.x,
                                       corners[0][0].y + (13 / 6) * axis212.y);
      cv::Point2f axis33 = cv::Point2f(corners[0][3].x + (13 / 6) * axis243.x,
                                       corners[0][3].y + (13 / 6) * axis243.y);
      cv::Point2f axis34 = cv::Point2f(corners[0][3].x + (7 / 6) * axis243.x,
                                       corners[0][3].y + (7 / 6) * axis243.y);
      corners3[0] = axis31;
      corners3[1] = axis32;
      corners3[2] = axis33;
      corners3[3] = axis34;
      corners_order[3] = corners3;
    } else if (ids[0] == 3) {
      std::vector<cv::Point2f> corners1;
      corners1.resize(4);
      cv::Point2f axis341 = cv::Point2f(corners[0][0].x - corners[0][3].x,
                                        corners[0][0].y - corners[0][3].y);
      cv::Point2f axis332 = cv::Point2f(corners[0][1].x - corners[0][2].x,
                                        corners[0][1].y - corners[0][2].y);
      cv::Point2f axis11 = cv::Point2f(corners[0][3].x + (13 / 6) * axis341.x,
                                       corners[0][3].y + (13 / 6) * axis341.y);
      cv::Point2f axis12 = cv::Point2f(corners[0][2].x + (13 / 6) * axis332.x,
                                       corners[0][2].y + (13 / 6) * axis332.y);
      cv::Point2f axis13 = cv::Point2f(corners[0][2].x + (7 / 6) * axis332.x,
                                       corners[0][2].y + (7 / 6) * axis332.y);
      cv::Point2f axis14 = cv::Point2f(corners[0][3].x + (7 / 6) * axis341.x,
                                       corners[0][3].y + (7 / 6) * axis341.y);
      corners1[0] = axis11;
      corners1[1] = axis12;
      corners1[2] = axis13;
      corners1[3] = axis14;

      std::vector<cv::Point2f> corners2;
      corners2.resize(4);
      cv::Point2f axis321 = cv::Point2f(corners[0][0].x - corners[0][1].x,
                                        corners[0][0].y - corners[0][1].y);
      cv::Point2f axis334 = cv::Point2f(corners[0][3].x - corners[0][2].x,
                                        corners[0][3].y - corners[0][2].y);
      cv::Point2f axis21 = cv::Point2f(corners[0][1].x + (13 / 6) * axis321.x,
                                       corners[0][1].y + (13 / 6) * axis321.y);
      cv::Point2f axis22 = cv::Point2f(corners[0][1].x + (7 / 6) * axis321.x,
                                       corners[0][1].y + (7 / 6) * axis321.y);
      cv::Point2f axis23 = cv::Point2f(corners[0][2].x + (7 / 6) * axis334.x,
                                       corners[0][2].y + (7 / 6) * axis334.y);
      cv::Point2f axis24 = cv::Point2f(corners[0][2].x + (13 / 6) * axis334.x,
                                       corners[0][2].y + (13 / 6) * axis334.y);
      corners2[0] = axis21;
      corners2[1] = axis22;
      corners2[2] = axis23;
      corners2[3] = axis24;

      std::vector<cv::Point2f> corners0;
      corners0.resize(4);
      cv::Point2f axis121 = cv::Point2f(corners1[0].x - corners1[1].x,
                                        corners1[0].y - corners1[1].y);
      cv::Point2f axis134 = cv::Point2f(corners1[3].x - corners1[2].x,
                                        corners1[3].y - corners1[2].y);
      cv::Point2f axis01 = cv::Point2f(corners1[1].x + (13 / 6) * axis121.x,
                                       corners1[1].y + (13 / 6) * axis121.y);
      cv::Point2f axis02 = cv::Point2f(corners1[1].x + (7 / 6) * axis121.x,
                                       corners1[1].y + (7 / 6) * axis121.y);
      cv::Point2f axis03 = cv::Point2f(corners1[2].x + (7 / 6) * axis134.x,
                                       corners1[2].y + (7 / 6) * axis134.y);
      cv::Point2f axis04 = cv::Point2f(corners1[2].x + (13 / 6) * axis134.x,
                                       corners1[2].y + (13 / 6) * axis134.y);
      corners0[0] = axis01;
      corners0[1] = axis02;
      corners0[2] = axis03;
      corners0[3] = axis04;
      corners_order[0] = corners0;
      corners_order[1] = corners1;
      corners_order[2] = corners2;

      corners_order[3] = corners[0];
    }

    cv::Vec3d rvec;
    cv::Vec3d tvec;
    std::vector<cv::Point2f> corner = _extractMarkerCorners(corners_order);
    std::vector<cv::Point3f> objPoints =
        _generate4MarkerWorldCoords(mark_length);
    auto compute = cv::solvePnP(objPoints, corner, camera_matrix,
                                distoration_coeff, rvec, tvec);
    if (compute == false) {
      return false;
    }
    Pose3D pose_out = _convertOpenCVPoseToPose3D(rvec, tvec);
    mark_pose.push_back(pose_out);

    float length = mark_length / 4;
    int thickness = 2;
    cv::drawFrameAxes(draw_image, camera_matrix, distoration_coeff, rvec, tvec,
                      length, thickness);
    setDrawFrame(draw_image);
    setMarkPoints(corner);
    setMarkPose(mark_pose);
    return true;
  } else {
    return false;
  }
}