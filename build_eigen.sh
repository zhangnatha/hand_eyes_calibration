#!/bin/bash

# 设置退出条件：任何命令失败时退出
set -e

# 定义安装路径
SOURCE_DIR="$(pwd)"
INSTALL_DIR="$(pwd)/3rdparty/eigen"
EIGEN_VERSION="3.4.0"

# 打印开始信息
echo "开始安装 Eigen ${EIGEN_VERSION} 到 ${INSTALL_DIR}"

# 1. 安装依赖
echo "安装必要的依赖..."
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y git build-essential -y

# 2. 下载 Eigen 源码
echo "下载 Eigen ${EIGEN_VERSION} 源码..."
mkdir -p "${SOURCE_DIR}"
cd "${SOURCE_DIR}"
if [ ! -d "eigen-${EIGEN_VERSION}" ]; then
    wget https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz
    tar -xzf eigen-${EIGEN_VERSION}.tar.gz
fi
cd eigen-${EIGEN_VERSION}

# 3. 清空旧构建目录
echo "清空旧构建目录..."
rm -rf build
mkdir -p build && cd build

# 4. 配置编译环境
echo "配置 CMake..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"

# 5. 安装 Eigen
echo "安装 Eigen 到 ${INSTALL_DIR}..."
make install

# 6. 验证安装
echo "验证安装..."
if [ -d "${INSTALL_DIR}/include/eigen3" ] && [ -f "${INSTALL_DIR}/include/eigen3/Eigen/Core" ]; then
    echo "安装成功！Eigen 头文件位于 ${INSTALL_DIR}/include/eigen3"
else
    echo "安装失败，请检查日志！"
    exit 1
fi

# 7. 清除
echo "清除..."
cd ${SOURCE_DIR}
rm -rf eigen-${EIGEN_VERSION}.tar.gz
rm -rf eigen-${EIGEN_VERSION}
echo "Eigen ${EIGEN_VERSION} 已成功安装到 ${INSTALL_DIR}"
