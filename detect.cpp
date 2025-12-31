#include "common/ArucoMarkerDetector.h"
#include "common/CoordinateTransformer.h"
#include "common/RealSenseCamera.h"

int main(int argc, char *argv[]) {
  std::string xml_path;
  std::string image_path;
  std::string mode = "image";
  bool show_help = false;

  if (argc == 1) {
    std::cout << "Error: No parameters provided. Use --help for usage."
              << std::endl;
    return 1;
  }

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help") {
      show_help = true;
      break;
    } else {
      size_t eq_pos = arg.find('=');
      if (eq_pos != std::string::npos) {
        std::string key = arg.substr(0, eq_pos);
        std::string value = arg.substr(eq_pos + 1);
        if (!value.empty() && value.front() == '"' && value.back() == '"') {
          value = value.substr(1, value.size() - 2);
        }
        if (key == "xml") {
          xml_path = value;
        } else if (key == "image") {
          image_path = value;
        } else if (key == "mode") {
          if (value != "camera" && value != "image") {
            std::cout << "Invalid mode: " << value
                      << ". Must be 'camera' or 'image'." << std::endl;
            return 1;
          }
          mode = value;
        } else {
          std::cout << "Unknown option: " << key << std::endl;
          return 1;
        }
      } else {
        std::cout << "Invalid argument format. Expected key=\"value\""
                  << std::endl;
        return 1;
      }
    }
  }

  if (show_help) {
    std::cout << "Usage: detect [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  xml=\"path/to/calibration.xml\" : Path to camera "
                 "calibration XML file. (required)"
              << std::endl;
    std::cout << "  mode=\"camera\" or \"image\" : Operation mode. Default is "
                 "\"image\"."
              << std::endl;
    std::cout << "  image=\"path/to/image.png\" : Path to input image "
                 "(required if mode=\"image\")."
              << std::endl;
    std::cout << "  --help : Display this help message." << std::endl;
    return 0;
  }

  if (xml_path.empty()) {
    std::cout
        << "Error: Missing required parameter 'xml'. Use --help for usage."
        << std::endl;
    return 1;
  }
  if (mode == "image" && image_path.empty()) {
    std::cout << "Error: Missing required parameter 'image' in image mode. Use "
                 "--help for usage."
              << std::endl;
    return 1;
  }

  ArucoMarkerDetector detector;
  std::cout << "Starting real-time pose estimation...\n";
  bool use_camera = (mode == "camera");
  RealSenseCamera camera;
  if (use_camera) {
    int width = 640;
    int height = 480;
    int fps = 60;
    camera.start(width, height, fps);
    camera.printIntrinsics();
  }

#if 0
    cv::FileStorage fs(xml_path, cv::FileStorage::READ);
    cv::Mat camera_matrix, distortion_coeffs;
    fs["K"] >> camera_matrix;
    fs["distortion"] >> distortion_coeffs;
    fs.release();
#else
  cv::Mat camera_matrix = camera.getColorCameraMatrix();
  cv::Mat distortion_coeffs = camera.getColorDistCoeffs();
#endif

  bool running = true;
  while (running) {
    cv::Mat color_frame;
    if (use_camera) {
      if (camera.hasNewFrame()) {
        color_frame = camera.getLatestColor();
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
    } else {
      color_frame = cv::imread(image_path);
      if (color_frame.empty()) {
        std::cout << "Error: Failed to load image from " << image_path
                  << std::endl;
        return 1;
      }
    }

#if 0
        cv::Mat depth_frame = camera.getLatestDepth();
        cv::Mat depth8;
        depth_frame.convertTo(depth8, CV_8U, 255.0 / 1000.0);
        cv::applyColorMap(depth8, depth8, cv::COLORMAP_JET);
        depth8.setTo(0, depth_frame == 0);
        cv::imshow("Depth (Aligned)", depth8);
#endif
    detector.setInputImage(color_frame);
    detector.setMarkLength(50);
    detector.setCameraMatrix(camera_matrix);
    detector.setDistorationCoeff(distortion_coeffs);
    bool detect_success = detector.detectMarkersAndEstimatePose();

    if (detect_success && !detector.getMarkPose().empty()) {
      auto mark_pose = detector.getMarkPose();
      std::vector<cv::Point2f> corners = detector.getMarkPoints();
      cv::Mat draw_pic = detector.getDrawFrame();

      std::cout << "[Camera]Pose in camera coordinate system: "
                << mark_pose[0].x() << ", " << mark_pose[0].y() << ", "
                << mark_pose[0].z() << ", " << mark_pose[0].rx() << ", "
                << mark_pose[0].ry() << ", " << mark_pose[0].rz() << "\n";

      // --- Load Hand-Eye Calibration ---
      std::string extrinsic_path = "calib_extrinsic.xml";
      cv::FileStorage fs_ext(extrinsic_path, cv::FileStorage::READ);
      cv::Mat R_ext, t_ext;
      Pose3D hand_eye_pose;

      if (fs_ext.isOpened()) {
        fs_ext["R"] >> R_ext;
        fs_ext["t"] >> t_ext;
        fs_ext.release();

        if (!R_ext.empty() && !t_ext.empty()) {
          cv::Mat H = cv::Mat::eye(4, 4, CV_64F);
          R_ext.copyTo(H(cv::Rect(0, 0, 3, 3)));
          t_ext.copyTo(H(cv::Rect(3, 0, 1, 3)));
          hand_eye_pose = CoordinateTransformer::fromHomogeneousMatrix(H);
        } else {
          std::cerr << "Error: Invalid data in " << extrinsic_path
                    << ". Using fallback pose." << std::endl;
        }
      } else {
        std::cerr << "Warning: Could not open " << extrinsic_path
                  << ". Using default fallback pose." << std::endl;
      }

      cv::Mat hand_eye_mat =
          CoordinateTransformer::toHomogeneousMatrix(hand_eye_pose);
      cv::Mat pose_camera_mat =
          CoordinateTransformer::toHomogeneousMatrix(mark_pose[0]);
      cv::Mat pose_base_mat = hand_eye_mat * pose_camera_mat;
      Pose3D pose_base =
          CoordinateTransformer::fromHomogeneousMatrix(pose_base_mat);
      std::cout << "[Robot]Pose in base coordinate system: " << pose_base.x()
                << ", " << pose_base.y() << ", " << pose_base.z() << ", "
                << pose_base.rx() << ", " << pose_base.ry() << ", "
                << pose_base.rz() << "\n";

      cv::imshow("6DoF Pose", draw_pic);
      if (cv::waitKey(1) == 27)
        break;
    } else {
      cv::imshow("6DoF Pose", color_frame);
      if (cv::waitKey(1) == 27)
        break;
    }

    if (!use_camera) {
      running = false;
      cv::waitKey(0);
    }
  }

  return 0;
}