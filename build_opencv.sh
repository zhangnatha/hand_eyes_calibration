#!/bin/bash

# 设置退出条件：任何命令失败时退出
set -e

# 定义安装路径
SOURCE_DIR="$(pwd)"
INSTALL_DIR="$(pwd)/3rdparty/opencv"
OPENCV_VERSION="4.5.5"

# 打印开始信息
echo "开始编译并安装 OpenCV ${OPENCV_VERSION} 到 ${INSTALL_DIR}"

# 1. 安装依赖
echo "安装必要的依赖..."
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y git build-essential libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev \
    libjpeg-dev libpng-dev libtiff-dev zlib1g-dev python3-dev python3-numpy -y

# 2. 下载 OpenCV 源码
echo "下载 OpenCV ${OPENCV_VERSION} 源码..."
mkdir -p "${SOURCE_DIR}"
cd "${SOURCE_DIR}"
if [ ! -d "opencv-${OPENCV_VERSION}" ]; then
    wget https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.tar.gz -O opencv-${OPENCV_VERSION}.tar.gz
    tar -xzf opencv-${OPENCV_VERSION}.tar.gz
fi

# 3. 下载 OpenCV contrib 模块（可选）
echo "下载 OpenCV contrib 模块..."
cd "${SOURCE_DIR}"
if [ ! -d "opencv_contrib-${OPENCV_VERSION}" ]; then
    wget https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.tar.gz -O opencv_contrib-${OPENCV_VERSION}.tar.gz
    tar -xzf opencv_contrib-${OPENCV_VERSION}.tar.gz
fi

# 4. 配置编译环境
cd opencv-${OPENCV_VERSION}
echo "配置 CMake..."
mkdir -p build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DOPENCV_EXTRA_MODULES_PATH="${SOURCE_DIR}/opencv_contrib-${OPENCV_VERSION}/modules" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DWITH_OPENGL=ON \
    -DWITH_V4L=OFF \
    -DWITH_LIBV4L=OFF \
    -DWITH_XVID=OFF \
    -DWITH_FFMPEG=ON \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"

# 5. 编译源码
echo "编译 OpenCV..."
make -j$(nproc)

# 6. 安装 OpenCV
echo "安装 OpenCV 到 ${INSTALL_DIR}..."
make install

# 7. 验证安装
echo "验证安装..."
if [ -f "${INSTALL_DIR}/lib/cmake/opencv4/OpenCVConfig.cmake" ] && [ -f "${INSTALL_DIR}/lib/libopencv_core.so" ]; then
    echo "安装成功！OpenCVConfig.cmake 位于 ${INSTALL_DIR}/lib/cmake/opencv4"
else
    echo "安装失败，请检查日志！"
    exit 1
fi

# 8. 清除
echo "清除..."
cd ${SOURCE_DIR}
rm -rf opencv_contrib-${OPENCV_VERSION}
rm -rf opencv-${OPENCV_VERSION}
rm -rf opencv_contrib-${OPENCV_VERSION}.tar.gz
rm -rf opencv-${OPENCV_VERSION}.tar.gz
echo "OpenCV ${OPENCV_VERSION} 已成功安装到 ${INSTALL_DIR}"
