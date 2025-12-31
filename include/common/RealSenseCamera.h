#ifndef REALSENSECAMERA_H
#define REALSENSECAMERA_H

#include <chrono>               // Potential delay in main loop
#include <iostream>             // Console output
#include <librealsense2/rs.hpp> // RealSense SDK header
#include <mutex>                // Thread-safe shared data
#include <opencv2/opencv.hpp>   // OpenCV header
#include <thread>               // Sleep in main loop
#include <vector>

// RealSenseCamera 类，用于初始化和帧回调
class RealSenseCamera {
private:
  rs2::pipeline pipe;           // RealSense 管道
  rs2::align align_to_color;    // 颜色对齐器
  cv::Mat latest_color;         // 最新颜色图
  cv::Mat latest_depth_aligned; // 最新对齐后的深度图
  std::mutex data_mutex;        // 数据互斥锁
  bool has_new_frame = false;   // 是否有新帧
  int frame_count = 0;          // 帧计数

  // 当新帧到达时触发的回调函数
  void frame_callback(const rs2::frame &frame) {
    if (auto frameset = frame.as<rs2::frameset>()) {
      frame_count++;
      if (frame_count <= 20)
        return;

      auto aligned = align_to_color.process(frameset);

      if (auto color = aligned.get_color_frame()) {
        cv::Mat img(cv::Size(color.get_width(), color.get_height()), CV_8UC3,
                    (void *)color.get_data(), cv::Mat::AUTO_STEP);
        std::lock_guard<std::mutex> lock(data_mutex);
        latest_color = img.clone();
        has_new_frame = true;
      }

      if (auto depth = aligned.get_depth_frame()) {
        cv::Mat dep(cv::Size(depth.get_width(), depth.get_height()), CV_16U,
                    (void *)depth.get_data(), cv::Mat::AUTO_STEP);
        std::lock_guard<std::mutex> lock(data_mutex);
        latest_depth_aligned = dep.clone();
        has_new_frame = true;
      }
    }
  }

public:
  // 构造函数：初始化管道
  RealSenseCamera() : align_to_color(RS2_STREAM_COLOR) {}
  ~RealSenseCamera() { stop(); }

  /**
   * @brief 启动相机流
   * @param width [in] 图像宽度
   * @param height [in] 图像高度
   * @param fps [in] 帧率
   */
  void start(int width, int height, int fps) {
    try {
      rs2::config cfg;
      // 配置流（颜色和深度）
      cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8,
                        fps); // 颜色流 (BGR8)
      cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16,
                        fps); // 深度流 (可选)

      // 使用回调 lambda 启动管道
      pipe.start(cfg, [this](const rs2::frame &frame) {
        this->frame_callback(frame);
      });
      std::cout << "RealSense 相机已启动。" << std::endl;
    } catch (const rs2::error &e) {
      std::cerr << "RealSense 错误调用 " << e.get_failed_function() << "("
                << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "异常: " << e.what() << std::endl;
    }
  }

  /**
   * @brief 停止相机流
   */
  void stop() {
    if (pipe.poll_for_frames(nullptr)) {
      pipe.stop();
      std::cout << "RealSense 相机已停止。" << std::endl;
    }
  }

  /**
   * @brief 获取最新颜色图
   * @return OpenCV 矩阵格式的颜色图
   */
  cv::Mat getLatestColor() {
    std::lock_guard<std::mutex> lock(data_mutex);
    has_new_frame = false;
    return latest_color.clone();
  }

  /**
   * @brief 获取最新深度图
   * @return OpenCV 矩阵格式的深度图
   */
  cv::Mat getLatestDepth() {
    std::lock_guard<std::mutex> lock(data_mutex);
    has_new_frame = false;
    return latest_depth_aligned.clone();
  }

  /**
   * @brief 检查是否有新帧
   * @return 如果有新帧返回 true，否则返回 false
   */
  bool hasNewFrame() {
    std::lock_guard<std::mutex> lock(data_mutex);
    return has_new_frame;
  }

  /**
   * @brief 打印相机内参信息
   */
  void printIntrinsics() {
    try {
      auto profile = pipe.get_active_profile();
      auto color_stream =
          profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
      auto depth_stream =
          profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();

      auto color_intrinsics = color_stream.get_intrinsics();
      auto depth_intrinsics = depth_stream.get_intrinsics();

      std::cout << "颜色相机内参:" << std::endl;
      std::cout << "  宽度: " << color_intrinsics.width
                << ", 高度: " << color_intrinsics.height << std::endl;
      std::cout << "  fx: " << color_intrinsics.fx
                << ", fy: " << color_intrinsics.fy << std::endl;
      std::cout << "  ppx: " << color_intrinsics.ppx
                << ", ppy: " << color_intrinsics.ppy << std::endl;
      std::cout << "  畸变模型: " << color_intrinsics.model << ", 畸变系数: ";
      for (int i = 0; i < 5; ++i) {
        std::cout << color_intrinsics.coeffs[i] << " ";
      }
      std::cout << std::endl;

      std::cout << "深度相机内参:" << std::endl;
      std::cout << "  宽度: " << depth_intrinsics.width
                << ", 高度: " << depth_intrinsics.height << std::endl;
      std::cout << "  fx: " << depth_intrinsics.fx
                << ", fy: " << depth_intrinsics.fy << std::endl;
      std::cout << "  ppx: " << depth_intrinsics.ppx
                << ", ppy: " << depth_intrinsics.ppy << std::endl;
      std::cout << "  畸变模型: " << depth_intrinsics.model << ", 畸变系数: ";
      for (int i = 0; i < 5; ++i) {
        std::cout << depth_intrinsics.coeffs[i] << " ";
      }
      std::cout << std::endl;
    } catch (const rs2::error &e) {
      std::cerr << "获取内参时出错: " << e.what() << std::endl;
    }
  }

  /**
   * @brief 获取颜色相机内参矩阵
   * @return 3x3 相机内参矩阵
   */
  cv::Mat getColorCameraMatrix() const {
    try {
      auto profile = pipe.get_active_profile();
      auto stream =
          profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
      auto intr = stream.get_intrinsics();

      cv::Mat K = (cv::Mat_<double>(3, 3) << intr.fx, 0, intr.ppx, 0, intr.fy,
                   intr.ppy, 0, 0, 1);
      return K;
    } catch (...) {
      std::cerr << "警告：获取颜色内参失败，使用默认值！" << std::endl;
      return (cv::Mat_<double>(3, 3) << 608, 0, 320, 0, 608, 240, 0, 0, 1);
    }
  }

  /**
   * @brief 获取颜色相机畸变系数
   * @return 1x5 畸变系数矩阵
   */
  cv::Mat getColorDistCoeffs() const {
    try {
      auto profile = pipe.get_active_profile();
      auto stream =
          profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
      auto intr = stream.get_intrinsics();

      cv::Mat dist = cv::Mat(1, 5, CV_64F);
      for (int i = 0; i < 5; ++i) {
        dist.at<double>(0, i) = intr.coeffs[i];
      }
      return dist;
    } catch (...) {
      std::cerr << "警告：获取畸变系数失败，返回全 0！" << std::endl;
      return cv::Mat::zeros(1, 5, CV_64F);
    }
  }
  std::vector<double> calibration_error_; // 标定误差
};

#endif // REALSENSECAMERA_H
