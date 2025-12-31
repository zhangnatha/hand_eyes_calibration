#include "calibration/01_intrinsic/IntrinsicCalibrator.h"
#include "calibration/02_hand_eye_2d_4point/FourPointCalibrator.h"
#include "calibration/03_hand_eye_2d_12point/TwelvePointCalibrator.h"
#include "calibration/04_hand_eye_3d_ball/BallCalibrator.h"
#include "calibration/05_hand_eye_3d_board_img/BoardImageCalibrator.h"
#include "calibration/06_hand_eye_3d_board_cloud/BoardCloudCalibrator.h"
#include "calibration/hand_eye_3d_base/RobotCameraCalibrator3D.h"
#include "common/CoordinateTransformer.h"
#include <fstream>
#include <iomanip>
#include <vector>

#define RUN_INTRINSIC 1
#define RUN_FOUR_POINT_2D 1
#define RUN_TWELVE_POINT_2D 1
#define RUN_HAND_EYE_3D_BALL 1
#define RUN_HAND_EYE_3D_BOARD_IMG 1
#define RUN_HAND_EYE_3D_BOARD_CLOUD 0

struct Logger {
  static void info(const std::string &msg) {
    std::cout << "\033[1;36m[INFO] " << msg << "\033[0m" << std::endl;
  }

  static void success(const std::string &msg) {
    std::cout << "\033[1;32m[SUCCESS] " << msg << "\033[0m" << std::endl;
  }

  static void warn(const std::string &msg) {
    std::cout << "\033[1;33m[WARN] " << msg << "\033[0m" << std::endl;
  }

  static void error(const std::string &msg) {
    std::cerr << "\033[1;31m[ERROR] " << msg << "\033[0m" << std::endl;
  }

  static void section(const std::string &msg) {
    std::cout << "\n\033[1;35m"
              << "============================================================="
                 "========"
              << "\n   " << msg << "\n"
              << "============================================================="
                 "========"
              << "\033[0m" << std::endl;
  }
};

int main() {
  std::cout << std::fixed << std::setprecision(4);
  Logger::section("Industrial Camera + Robot Hand-Eye Calibration Program");
  std::cout << "Included: 2.5D Hand-Eye | 2D Four-Point | 2D Twelve-Point High "
               "Precision Calibration\n"
            << std::endl;

#if RUN_INTRINSIC
  CalibConfig cfg;

  // --- Preset 1: Asymmetric Circles (Recommended: 4x11) ---
  // cfg.pattern = CIRCLES_ASYM;
  // cfg.cols = 4;
  // cfg.rows = 11;
  // cfg.interval_mm = 25; // Standard A4 print spacing

  // --- Preset 2: Chessboard (Standard: 7x6) ---
  // cfg.pattern = CHESSBOARD;
  // cfg.cols = 7;
  // cfg.rows = 6;
  // cfg.interval_mm = 25; // Standard A4 print spacing

  // --- Current Active Config (Default) ---
  cfg.pattern = CIRCLES_ASYM;
  cfg.calib_type = CalibrationType3D::ETH;
  cfg.cols = 4;
  cfg.rows = 5;
  cfg.interval_mm = 20;         // 标定板圆心间距
  cfg.marker_length_mm = 50.0f; // 标定板marker长度

  std::vector<std::string> intrinsic_img_files = {
      "../assert/camera/10.png", "../assert/camera/11.png",
      "../assert/camera/12.png", "../assert/camera/13.png",
      "../assert/camera/14.png", "../assert/camera/15.png",
      "../assert/camera/16.png", "../assert/camera/17.png",
      "../assert/camera/18.png", "../assert/camera/19.png",
      "../assert/camera/20.png", "../assert/camera/21.png",
      "../assert/camera/22.png", "../assert/camera/23.png",
      "../assert/camera/24.png", "../assert/camera/25.png",
      "../assert/camera/26.png", "../assert/camera/27.png",
      "../assert/camera/28.png", "../assert/camera/29.png"};
  std::vector<std::string> extrinsic_img_files = intrinsic_img_files;
  const std::vector<Pose3D> robot_poses = {
      {211.892, -415.103, -536.213, -91.135, 44.296, 11.726},
      {217.353, -412.223, -530.437, -89.057, 44.896, 13.651},
      {235.681, -408.397, -527.87, -87.607, 45.096, 15.771},
      {235.678, -411.597, -527.868, -84.762, 46.641, 21.056},
      {252.301, -427.281, -524.428, -82.312, 47.657, 23.928},
      {251.664, -412.156, -518.114, -80.204, 48.466, 27.648},
      {250.344, -422.522, -520.923, -78.108, 49.324, 32.218},
      {252.918, -401.931, -541.12, -83.448, 46.17, 23.515},
      {251.206, -386.262, -542.755, -85.8, 45.12, 20.078},
      {223.281, -378.458, -542.76, -88.365, 44.261, 16.83},
      {219.702, -418.79, -536.205, -89.737, 50.569, 12.734},
      {217.392, -412.203, -530.434, -91.857, 39.008, 18.558},
      {235.686, -408.395, -527.868, -79.344, 39.103, 18.983},
      {235.683, -411.597, -535.154, -79.481, 39.239, 23.953},
      {245.722, -414.54, -523.743, -84.142, 50.207, 28.574},
      {256.122, -412.152, -526.174, -79.561, 44.677, 33.859},
      {238.346, -422.975, -530.894, -84.73, 48.306, 25.753},
      {252.926, -401.932, -536.669, -83.512, 38.762, 28.218},
      {251.213, -386.262, -542.757, -82.352, 48.152, 39.276},
      {229.352, -374.528, -536.877, -86.592, 38.116, 17.081}};

  Logger::section("Category 1: Intrinsic Calibration");

  IntrinsicCalibrator calib_obj;
  std::ifstream f_intrinsic(cfg.xml_intrinsic_1st);
  if (!calib_obj.intrinsicCalibrate(cfg, intrinsic_img_files)) {
    Logger::error("Intrinsic calibration failed!");
    return -1;
  }
  Logger::success("Intrinsic calibration success!");
#endif

#if RUN_HAND_EYE_3D_BOARD_IMG
  Logger::section(
      "Category 2: 3D Camera Hand-Eye Calibration - Based on Board Image");

  std::ifstream f_extrinsic(cfg.xml_extrinsic);
  Logger::info("Starting hand-eye calibration...");
  BoardImageCalibrator board_calib_obj;
  board_calib_obj.setConfig(cfg);
  board_calib_obj.setImageFiles(extrinsic_img_files);
  board_calib_obj.setRobotPoses(robot_poses);
  board_calib_obj.setCalibrationType(cfg.calib_type);
  // 设置欧拉角类型 (示例: XYZ 外旋)
  board_calib_obj.setRPYType(RPY::RPYType::XYZ, RPY::ReferenceType::EXTRINSIC);

  if (!board_calib_obj.runCalibration()) {
    Logger::error("3D Hand-Eye Calibration failed!");
    return -1;
  }
  Logger::success("3D Hand-Eye Calibration success!");
  std::cout << "\n";
  Pose3D hand_eye_pose = board_calib_obj.getResultPose();
  auto errors = board_calib_obj.getErrors();
  auto markers = board_calib_obj.getMarkerPoints();
  auto marker_success = board_calib_obj.getMarkerSuccess();

  std::cout << "[INFO] Index | Robot(x,y,z,rx,ry,rz) | Marker(x,y,z) | Error\n";
  std::cout
      << "--------------------------------------------------------------\n";
  for (size_t i = 0; i < robot_poses.size(); ++i) {
    Pose3D p = robot_poses[i];
    std::cout << std::setw(3) << i << " | " << std::setw(10) << p.x() << ", "
              << std::setw(10) << p.y() << ", " << std::setw(10) << p.z()
              << ", " << std::setw(10) << p.rx() << ", " << std::setw(10)
              << p.ry() << ", " << std::setw(10) << p.rz() << " | ";

    if (i < marker_success.size() && marker_success[i]) {
      std::cout << std::setw(10) << markers[i].x() << ", " << std::setw(10)
                << markers[i].y() << ", " << std::setw(10) << markers[i].z();
    } else {
      std::cout << std::setw(10) << "N/A" << ", " << std::setw(10) << "N/A"
                << ", " << std::setw(10) << "N/A";
    }

    std::cout << " | ";
    if (i < errors.size() && errors[i] >= 0)
      std::cout << std::setw(10) << errors[i];
    else
      std::cout << std::setw(10) << "N/A";
    std::cout << "\n";
  }

  double errorSum = 0.0;
  int errCount = 0;
  for (auto error : errors) {
    if (error >= 0) {
      errorSum += error;
      errCount++;
    }
  }
  double avg = errCount == 0 ? 0 : errorSum / errCount;

  double maxE = 0, minE = 0;
  bool first = true;
  for (auto error : errors) {
    if (error < 0)
      continue;
    if (first) {
      maxE = minE = error;
      first = false;
    } else {
      maxE = std::max(maxE, error);
      minE = std::min(minE, error);
    }
  }

  Logger::success(
      "3D Hand-Eye Calibration successful! Avg Error: " + std::to_string(avg) +
      ", Max: " + std::to_string(maxE) + ", Min: " + std::to_string(minE));

  std::cout << "[INFO] hand_eyes Homogeneous Matrix (Eigen format):\n"
            << hand_eye_pose.toTransform3D().eigen() << "\n"
            << std::endl;

  std::cout << "[RESULT] hand_eyes (Pose3D format): \n"
            << hand_eye_pose.x() << ", " << hand_eye_pose.y() << ", "
            << hand_eye_pose.z() << ", " << hand_eye_pose.rx() << ", "
            << hand_eye_pose.ry() << ", " << hand_eye_pose.rz() << std::endl;

  cv::Mat H_hand_eyes =
      CoordinateTransformer::toHomogeneousMatrix(hand_eye_pose);
  cv::Mat H_cam_to_gripper_R = H_hand_eyes(cv::Rect(0, 0, 3, 3)).clone();
  cv::Mat H_cam_to_gripper_t = H_hand_eyes(cv::Rect(3, 0, 1, 3)).clone();

  Logger::info("Hand-eye calibration results (R | t) saved to: " +
               cfg.xml_extrinsic);

  cv::FileStorage fs_hand_eye(cfg.xml_extrinsic, cv::FileStorage::WRITE);
  fs_hand_eye << "R" << H_cam_to_gripper_R << "t" << H_cam_to_gripper_t;
  fs_hand_eye.release();
  Logger::success("3D Hand-Eye Calibration complete!");
#endif

#if RUN_FOUR_POINT_2D
  Logger::section("Category 3: 2D Camera Hand-Eye Calibration - 4-Point Pin");

  std::vector<cv::Point2d> img_points = {
      {2035, 1093}, {3129, 1087}, {3130, 1549}, {2038, 1556}};
  std::vector<cv::Point2d> robot_points = {
      {800.94, 886.14}, {805.45, 570.15}, {671.78, 568.56}, {667.07, 884.8}};
  if (img_points.size() < 4 || robot_points.size() < 4)
    return 1;

  FourPointCalibrator fp_calib;
  cv::Mat hand_eyes_4p;
  if (!fp_calib.calibrate(img_points, robot_points, hand_eyes_4p)) {
    Logger::error("Four-point calibration failed!");
    return -1;
  }
  std::cout << "[INFO] hand_eyes Homogeneous Matrix (2x3):\n"
            << hand_eyes_4p << "\n"
            << std::endl;

  cv::Point2d test_img_pt = {2035, 1093};
  cv::Point2d robot_point = fp_calib.transform(test_img_pt, hand_eyes_4p);

  std::cout << "\033[1;32m[VERIFY] Four-point pin verification: Image("
            << test_img_pt.x << "," << test_img_pt.y << ") → Robot("
            << robot_point.x << ", " << robot_point.y << ")\033[0m\n"
            << std::endl;
#endif

#if RUN_TWELVE_POINT_2D
  Logger::section("Category 4: 2D Camera Hand-Eye Calibration - 12-Point");
  TwelvePointCalibrator calib12p_obj;
  TwelvePointCalibrator::CalibrationType2D hand_eye_type =
      TwelvePointCalibrator::CalibrationType2D::EIH;
  TwelvePointCalibrator::CameraInstallType install_type =
      TwelvePointCalibrator::CameraInstallType::SameToTCPZ;
  Pose2D registration_point = {1188, 468, 0};
  std::vector<Pose2D> img_points_12p = {
      {1188, 468, -0}, {1173, 467, -0}, {1195, 464, -0}, {1320, 474, -0},
      {1249, 593, 5},  {1292, 545, 7},  {1346, 632, 6},  {1439, 520, 5},
      {1482, 595, 4},  {1329, 437, 6},  {1239, 472, -0}, {1420, 550, -14}};
  std::vector<Pose2D> robot_poses_12p = {
      {0, 0, 0},    {0, 30, 0},  {30, 30, 0},   {30, 0, 0},
      {30, -30, 0}, {0, -30, 0}, {-30, -30, 0}, {-30, 0, 0},
      {-30, 30, 0}, {0, 0, -10}, {0, 0, 0},     {0, 0, 10}};
  calib12p_obj.setHandEyeType(hand_eye_type);
  calib12p_obj.setCameraInstallType(install_type);
  calib12p_obj.setRegistrationPoint(registration_point);
  calib12p_obj.setImageMarksPoints(img_points_12p);
  calib12p_obj.setRobotPoses(robot_poses_12p);

  calib12p_obj.runCalibration();

  auto errors_12p = calib12p_obj.getCalibrationErrors();
  std::cout << "[INFO] Index | Robot(dx,dy,rz) | Image(x,y,theta) | Error\n";
  std::cout << "---------------------------------------------------------"
               "---------------------\n";
  for (size_t i = 0; i < robot_poses_12p.size(); ++i) {
    Pose2D p_robot = robot_poses_12p[i];
    Pose2D p_img = img_points_12p[i];
    std::cout << std::setw(3) << i << " | " << std::setw(10) << p_robot.x()
              << ", " << std::setw(10) << p_robot.y() << ", " << std::setw(10)
              << p_robot.angle() << " | " << std::setw(10) << p_img.x() << ", "
              << std::setw(10) << p_img.y() << ", " << std::setw(10)
              << p_img.angle() << " | ";
    if (i < errors_12p.size())
      std::cout << std::setw(10) << errors_12p[i];
    else
      std::cout << std::setw(10) << "N/A";
    std::cout << "\n";
  }

  Logger::success("2D Twelve-point Calibration successful!");

  auto trans_pose_12p = calib12p_obj.getTransformPose();
  std::cout << "[INFO] Transform Pose (2x3 Affine Matrix):\n"
            << trans_pose_12p(0, 0) << ", " << trans_pose_12p(0, 1) << ", "
            << trans_pose_12p(0, 2) << "\n"
            << trans_pose_12p(1, 0) << ", " << trans_pose_12p(1, 1) << ", "
            << trans_pose_12p(1, 2) << "\n"
            << std::endl;

  auto raw_tool_center = calib12p_obj.getRawToolCenter();
  auto fitting_radius = calib12p_obj.getFittingRadius();
  std::cout << "[INFO] Circle Fitting Result: \n"
            << "center=(" << raw_tool_center[0] << ", " << raw_tool_center[1]
            << "), R=(" << fitting_radius << ")\n"
            << std::endl;

  auto tcp_center = calib12p_obj.getRotationCenter();
  std::cout << "[RESULT] TCP Rotation Center (relative to registration point): "
            << tcp_center[0] << ", " << tcp_center[1] << std::endl;

  Pose2D test_img = {1188, 468, 0};
  Pose2D register_point = calib12p_obj.getRegistrationPoint();
  Vector2d tool_center = calib12p_obj.getRotationCenter();
  Matrix<double, 2, 3> trans_pose = calib12p_obj.getTransformPose();
  Pose2D robot_out;
  calib12p_obj.transformImageToRobotPose(test_img, register_point, tool_center,
                                         trans_pose, robot_out);
  std::cout << "\033[1;32m[VERIFY] Twelve-point verification: Image("
            << test_img.x() << "," << test_img.y() << ") → Robot("
            << robot_out.x() << ", " << robot_out.y() << ", "
            << robot_out.angle() << "°)\033[0m\n"
            << std::endl;
#endif

#if RUN_HAND_EYE_3D_BALL
  Logger::section(
      "Category 5: 3D Camera Hand-Eye Calibration - Based on Ball Center XYZ");

  // Robot poses: x, y, z, rx, ry, rz
  std::vector<std::vector<double>> robot_poses_data = {
      {-1223.07, 1493.14, 774.351, -172.691, -14.074, 170.725},
      {-1105.25, 1370.07, 774.303, -171.213, -18.84, 153.076},
      {-1064.56, 1424.31, 808.086, 166.736, -18.035, 160.874},
      {-1154.64, 1396.67, 780.382, 168.416, -13.818, 164.482},
      {-1099.28, 1471.44, 637.932, -170.201, -0.875, 160.177},
      {-1215.18, 1368.72, 656.108, -173.238, 14.414, 173.631},
      {-1148.45, 1422.8, 730.753, -176.372, 10.735, 163.365},
      {-1132.76, 1358.11, 775.324, 178.924, 4.649, 171.376},
      {-1231.24, 1314.03, 762.005, 169.6, -10.535, 173.234},
      {-1186.72, 1408.22, 682.34, 174.96, -4.641, 165.91},
      {-1106.26, 1441.8, 686.117, 179.956, 0.57, 152.826}};

  // Marker points (pixel coordinates): x, y, z
  std::vector<std::vector<double>> marker_points_data = {
      {-107, -2.25, 632.75},   {-3.75, -39.25, 528},
      {48.75, -37, 627.5},     {-29.75, -14.25, 580.25},
      {46.75, 108.75, 584.5},  {-19.5, 59.75, 490.75},
      {21, 2.5, 563},          {38.75, -47.25, 525.75},
      {-89.25, -16.75, 487.5}, {-36, 78.25, 554.25},
      {39.25, 65.5, 575.5}};

  // Prepare data for calibration
  std::vector<Eigen::Vector3d> mark_points;
  std::vector<Pose3D> robot_poses_p3d;

  for (size_t i = 0; i < robot_poses_data.size(); ++i) {
    const auto &robot = robot_poses_data[i];
    const auto &marker = marker_points_data[i];

    robot_poses_p3d.push_back(
        Pose3D(robot[0], robot[1], robot[2], robot[3], robot[4], robot[5]));
    mark_points.push_back(Eigen::Vector3d(marker[0], marker[1], marker[2]));
  }

  // Perform calibration using BallCalibrator object
  BallCalibrator ball_calib;
  ball_calib.setRobotPoses(robot_poses_p3d);
  ball_calib.setMarkPoints(mark_points);
  ball_calib.setCalibrationType(CalibrationType3D::ETH);
  ball_calib.setRPYType(RPY::RPYType::XYZ, RPY::ReferenceType::EXTRINSIC);

  if (ball_calib.runCalibration()) {
    Pose3D result = ball_calib.getResultPose();
    auto errors = ball_calib.getErrors();

    std::cout
        << "[INFO] Index | Robot(x,y,z,rx,ry,rz) | Marker(x,y,z) | Error\n";
    std::cout
        << "--------------------------------------------------------------\n";
    for (size_t i = 0; i < mark_points.size(); ++i) {
      Pose3D p = robot_poses_p3d[i];
      std::cout << std::setw(3) << i << " | " << std::setw(10) << p.x() << ", "
                << std::setw(10) << p.y() << ", " << std::setw(10) << p.z()
                << ", " << std::setw(10) << p.rx() << ", " << std::setw(10)
                << p.ry() << ", " << std::setw(10) << p.rz() << " | "
                << std::setw(10) << mark_points[i].x() << ", " << std::setw(10)
                << mark_points[i].y() << ", " << std::setw(10)
                << mark_points[i].z() << " | ";
      if (i < errors.size() && errors[i] >= 0)
        std::cout << std::setw(10) << errors[i];
      else
        std::cout << std::setw(10) << "N/A";
      std::cout << "\n";
    }

    double avg_error = 0;
    for (auto e : errors)
      avg_error += e;
    avg_error /= errors.size();

    Logger::success("3D Ball-based Calibration successful! Avg Error: " +
                    std::to_string(avg_error));

    std::cout << "[INFO] hand_eyes Homogeneous Matrix (Eigen format):\n"
              << result.toTransform3D().eigen() << "\n"
              << std::endl;

    std::cout << "[RESULT] hand_eyes (Pose3D format): \n"
              << result.x() << ", " << result.y() << ", " << result.z() << ", "
              << result.rx() << ", " << result.ry() << ", " << result.rz()
              << std::endl;
  } else {
    Logger::warn("3D Ball-based Calibration failed.");
  }
#endif

#if RUN_HAND_EYE_3D_BOARD_CLOUD
  Logger::section("Category 6: 3D Camera Hand-Eye Calibration - Based on Board "
                  "Image + Cloud");
  std::vector<std::string> img_files = {"../assert/1.png", "../assert/2.png",
                                        "../assert/3.png", "../assert/4.png",
                                        "../assert/5.png"};
  std::vector<std::string> cloud_luts_files = {
      "../assert/lut/1.bin", "../assert/lut/2.bin", "../assert/lut/3.bin",
      "../assert/lut/4.bin", "../assert/lut/5.bin"};
  std::vector<std::string> cloud_pcd_files = {
      "../assert/cloud/1.pcd", "../assert/cloud/2.pcd", "../assert/cloud/3.pcd",
      "../assert/cloud/4.pcd", "../assert/cloud/5.pcd"};

  BoardCloudCalibrator cloud_calib;
  cloud_calib.setConfig(cfg);
  cloud_calib.setImageFiles(img_files);
  cloud_calib.setCloudLuts(cloud_luts_files);
  cloud_calib.setCloudFiles(cloud_pcd_files);
  cloud_calib.setRobotPoses(robot_poses);
  cloud_calib.setCalibrationType(cfg.calib_type);
  cloud_calib.setRPYType(RPY::RPYType::XYZ, RPY::ReferenceType::EXTRINSIC);

  if (cloud_calib.runCalibration()) {
    Pose3D result = cloud_calib.getResultPose();
    auto errors = cloud_calib.getErrors();
    auto markers = cloud_calib.getMarkerPoints();
    auto marker_success = cloud_calib.getMarkerSuccess();

    std::cout
        << "[INFO] Index | Robot(x,y,z,rx,ry,rz) | Marker(x,y,z) | Error\n";
    std::cout
        << "--------------------------------------------------------------\n";
    for (size_t i = 0; i < robot_poses.size(); ++i) {
      Pose3D p = robot_poses[i];
      std::cout << std::setw(3) << i << " | " << std::setw(10) << p.x() << ", "
                << std::setw(10) << p.y() << ", " << std::setw(10) << p.z()
                << ", " << std::setw(10) << p.rx() << ", " << std::setw(10)
                << p.ry() << ", " << std::setw(10) << p.rz() << " | ";

      if (i < marker_success.size() && marker_success[i]) {
        std::cout << std::setw(10) << markers[i].x() << ", " << std::setw(10)
                  << markers[i].y() << ", " << std::setw(10) << markers[i].z();
      } else {
        std::cout << std::setw(10) << "N/A" << ", " << std::setw(10) << "N/A"
                  << ", " << std::setw(10) << "N/A";
      }

      std::cout << " | ";
      if (i < errors.size() && errors[i] >= 0)
        std::cout << std::setw(10) << errors[i];
      else
        std::cout << std::setw(10) << "N/A";
      std::cout << "\n";
    }

    double errorSum = 0.0;
    int errCount = 0;
    for (auto error : errors) {
      if (error >= 0) {
        errorSum += error;
        errCount++;
      }
    }
    double avg = errCount == 0 ? 0 : errorSum / errCount;

    double maxE = 0, minE = 0;
    bool first = true;
    for (auto error : errors) {
      if (error < 0)
        continue;
      if (first) {
        maxE = minE = error;
        first = false;
      } else {
        maxE = std::max(maxE, error);
        minE = std::min(minE, error);
      }
    }

    Logger::success("3D Board Cloud Calibration successful! Avg Error: " +
                    std::to_string(avg) + ", Max: " + std::to_string(maxE) +
                    ", Min: " + std::to_string(minE));

    std::cout << "[INFO] hand_eyes Homogeneous Matrix (Eigen format):\n"
              << result.toTransform3D().eigen() << "\n"
              << std::endl;

    std::cout << "[RESULT] hand_eyes (Pose3D format): \n"
              << result.x() << ", " << result.y() << ", " << result.z() << ", "
              << result.rx() << ", " << result.ry() << ", " << result.rz()
              << std::endl;
  } else {
    Logger::warn("3D Board Cloud Calibration failed.");
  }
#endif

  Logger::section("All Calibrations Complete!");

  return 0;
}
