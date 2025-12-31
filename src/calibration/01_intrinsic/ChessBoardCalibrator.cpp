#include "calibration/01_intrinsic/ChessBoardCalibrator.h"
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>

void ChessBoardCalibrator::worldCoordinate(
    std::vector<cv::Point3f> &world_corners) {
  world_corners.clear();
  for (int i = 0; i < board_size_rows_; ++i) {
    if (i % 2 == 0) {
      for (int j = 0; j < board_size_cols_; ++j) {
        world_corners.push_back(cv::Point3f(j * board_size_interval_,
                                            (i * board_size_interval_) / 2, 0));
      }
    } else {
      for (int j = 0; j < board_size_cols_; ++j) {
        world_corners.push_back(
            cv::Point3f(j * board_size_interval_ + board_size_interval_ / 2,
                        (i * board_size_interval_) / 2, 0));
      }
    }
  }
}

int ChessBoardCalibrator::finderCircleCorners(
    cv::Mat input_image, cv::Mat &output_corners_image,
    std::vector<cv::Point2f> &pixel_corners,
    std::vector<cv::Point3f> &world_corners, int circle_area) {
  cv::Mat image;
  if (input_image.channels() != 1)
    cv::cvtColor(input_image, image, cv::COLOR_RGB2GRAY);
  else
    input_image.copyTo(image);

  cv::SimpleBlobDetector::Params params;
  params.blobColor = 255;
  params.filterByColor = true;
  params.maxArea = circle_area;
  params.minArea = 1200;
  params.filterByArea = true;

  params.minDistBetweenBlobs = 36;
  params.filterByInertia = true;
  params.minInertiaRatio = 0.5;
  params.minConvexity = 0.8;
  params.thresholdStep = 20;
  cv::Ptr<cv::FeatureDetector> blobDetector =
      cv::SimpleBlobDetector::create(params);
  cv::CirclesGridFinderParameters paramss;
  paramss.gridType = cv::CirclesGridFinderParameters::GridType::ASYMMETRIC_GRID;
  paramss.minDistanceToAddKeypoint = 45;
  paramss.edgeGain = 10;
  paramss.edgePenalty = 10;

  bool patternfound;
  try {
    patternfound = cv::findCirclesGrid(
        image, cv::Size(board_size_cols_, board_size_rows_), pixel_corners,
        cv::CALIB_CB_CLUSTERING | cv::CALIB_CB_ASYMMETRIC_GRID, blobDetector,
        paramss);
  } catch (cv::Exception &e) {
    cv::cvtColor(image, output_corners_image, cv::COLOR_GRAY2RGB);
    return 1;
  } catch (...) {
    cv::cvtColor(image, output_corners_image, cv::COLOR_GRAY2RGB);
    return 1;
  }

  if (!patternfound) {
    cv::cvtColor(image, output_corners_image, cv::COLOR_GRAY2RGB);
    return 1;
  }

  cv::cvtColor(image, output_corners_image, cv::COLOR_GRAY2RGB);
  cv::drawChessboardCorners(output_corners_image,
                            cv::Size(board_size_cols_, board_size_rows_),
                            pixel_corners, patternfound);

  setMarkPoints(pixel_corners);
  worldCoordinate(world_corners);
  setWorldPoints(world_corners);
  if (!patternfound || pixel_corners.size() != world_corners.size())
    return 1;

  return 0;
}

bool ChessBoardCalibrator::solverPnP(std::vector<cv::Point3f> world_corners,
                                     std::vector<cv::Point2f> pixel_corners,
                                     cv::Mat camera_matrix, cv::Mat dist_coeffs,
                                     Pose3D &pose) {
  try {
    std::cout << "[INFO]ChessBoardCalibrator::solverPnP start \n";
    cv::Mat rotation, translation;
    cv::solvePnP(world_corners, pixel_corners, camera_matrix, dist_coeffs,
                 rotation, translation);

    Eigen::Vector3d point;
    point(0) = translation.at<double>(0, 0);
    point(1) = translation.at<double>(0, 1);
    point(2) = translation.at<double>(0, 2);

    pose = {point[0], point[1], point[2], 0, 0, 0};

    std::cout << "pnp: " << point[0] << "，" << point[1] << "，" << point[2]
              << std::endl;
    std::cout << "[INFO]ChessBoardCalibrator::solverPnP end \n";
    return true;
  } catch (cv::Exception &e) {
    return false;
  }
}

bool ChessBoardCalibrator::computePose() { return true; }
