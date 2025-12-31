/** This file contains the lookup tables for the PC algorithm */

#ifndef PC_LUT_H
#define PC_LUT_H

#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <vector>

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

typedef std::vector<std::vector<int>> PointCloudRegLUT;

// PointCloudRegLUT 的存储与解析接口
class PointCloudRegLUTHandler {
public:
  // 构造函数
  PointCloudRegLUTHandler() = default;

  // 存储 PointCloudRegLUT 到文件（支持 std::shared_ptr 输入）
  bool saveToFile(const std::shared_ptr<PointCloudRegLUT> &pointcloudReg,
                  const std::string &filename, bool useBinary = true) const {
    if (!pointcloudReg) {
      throw std::runtime_error("PointCloudRegLUT shared_ptr is null");
    }
    saveImpl(*pointcloudReg, filename, useBinary);
    return true;
  }

  // 存储 PointCloudRegLUT 到文件（支持直接 PointCloudRegLUT 输入）
  bool saveToFile(const PointCloudRegLUT &pointcloudReg,
                  const std::string &filename, bool useBinary = true) const {
    saveImpl(pointcloudReg, filename, useBinary);
    return true;
  }

  // 从文件解析 PointCloudRegLUT
  PointCloudRegLUT loadFromFile(const std::string &filename,
                                bool useBinary = true) const {
    if (useBinary) {
      return loadBinary(filename);
    } else {
      return loadText(filename);
    }
  }

  /** @brief 根据2D图像坐标查找对应的3D点云点
   *  @param x 2D图像的x坐标
   *  @param y 2D图像的y坐标
   *  @param pointcloudReg 查找表，用于映射2D坐标到点云索引
   *  @param pc_out 点云数据
   *  @param point 返回对应的3D点
   *  @return 如果找到有效的点云点，返回true；否则返回false
   */
  /*
  bool getPoint3DFrom2D(int x, int y,
                      const std::shared_ptr<PointCloudRegLUT>& pointcloudReg,
                      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out,
                      pcl::PointXYZRGB& point) const {
      // 检查查找表是否有效
      if (!pointcloudReg || pointcloudReg->empty() || x < 0 || y < 0) {
          std::cerr << "Invalid lookup table or 2D coordinates" << std::endl;
          return false;
      }

      // 确保查找表中有对应的行和列
      if (y >= pointcloudReg->size() || x >= (*pointcloudReg)[y].size()) {
          std::cerr << "2D coordinates out of bounds in lookup table" <<
  std::endl; return false;
      }

      // 获取2D坐标对应的点云索引
      int pointcloud_index = (*pointcloudReg)[y][x];

      // 如果索引无效，返回false
      if (pointcloud_index == -1 || pointcloud_index >=
  static_cast<int>(pc_out->size())) { std::cerr << "No valid point cloud data
  found for the given 2D coordinates" << std::endl; return false;
      }

      // 返回点云点
      point = pc_out->at(pointcloud_index);
      return true;
  }
  */

private:
  // 内部实现：存储（共享逻辑）
  void saveImpl(const PointCloudRegLUT &pointcloudReg,
                const std::string &filename, bool useBinary) const {
    if (useBinary) {
      saveBinary(pointcloudReg, filename);
    } else {
      saveText(pointcloudReg, filename);
    }
  }

  // 二进制存储
  void saveBinary(const PointCloudRegLUT &pointcloudReg,
                  const std::string &filename) const {
    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs.is_open()) {
      throw std::runtime_error("Failed to open file for writing: " + filename);
    }

    // 写入行数和列数
    uint32_t rows = static_cast<uint32_t>(pointcloudReg.size());
    uint32_t cols =
        (rows > 0) ? static_cast<uint32_t>(pointcloudReg[0].size()) : 0;
    ofs.write(reinterpret_cast<const char *>(&rows), sizeof(rows));
    ofs.write(reinterpret_cast<const char *>(&cols), sizeof(cols));

    // 写入数据
    for (const auto &row : pointcloudReg) {
      ofs.write(reinterpret_cast<const char *>(row.data()),
                row.size() * sizeof(int));
    }

    ofs.close();
  }

  // 二进制解析
  PointCloudRegLUT loadBinary(const std::string &filename) const {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs.is_open()) {
      throw std::runtime_error("Failed to open file for reading: " + filename);
    }

    // 读取行数和列数
    uint32_t rows, cols;
    ifs.read(reinterpret_cast<char *>(&rows), sizeof(rows));
    ifs.read(reinterpret_cast<char *>(&cols), sizeof(cols));

    // 初始化 PointCloudRegLUT
    PointCloudRegLUT result(rows, std::vector<int>(cols));

    // 读取数据
    for (uint32_t i = 0; i < rows; ++i) {
      ifs.read(reinterpret_cast<char *>(result[i].data()), cols * sizeof(int));
    }

    ifs.close();
    return result;
  }

  // 文本存储
  void saveText(const PointCloudRegLUT &pointcloudReg,
                const std::string &filename) const {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
      throw std::runtime_error("Failed to open file for writing: " + filename);
    }

    // 写入行数和列数
    ofs << pointcloudReg.size() << " "
        << (pointcloudReg.empty() ? 0 : pointcloudReg[0].size()) << "\n";

    // 写入数据
    for (const auto &row : pointcloudReg) {
      for (const auto &val : row) {
        ofs << val << " ";
      }
      ofs << "\n";
    }

    ofs.close();
  }

  // 文本解析
  PointCloudRegLUT loadText(const std::string &filename) const {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
      throw std::runtime_error("Failed to open file for reading: " + filename);
    }

    // 读取行数和列数
    uint32_t rows, cols;
    ifs >> rows >> cols;

    // 初始化 PointCloudRegLUT
    PointCloudRegLUT result(rows, std::vector<int>(cols));

    // 读取数据
    for (uint32_t i = 0; i < rows; ++i) {
      for (uint32_t j = 0; j < cols; ++j) {
        ifs >> result[i][j];
      }
    }

    ifs.close();
    return result;
  }
};

#endif // PC_LUT_H