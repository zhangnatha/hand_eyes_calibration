#!/bin/bash

# 设置退出条件：任何命令失败时退出
set -e

# 定义安装路径
SOURCE_DIR="$(pwd)"
INSTALL_DIR="$(pwd)/3rdparty/camera_realsense"
SDK_VERSION="v2.54.2"

# 打印开始信息
echo "开始编译并安装 Intel RealSense SDK ${SDK_VERSION} 到 ${INSTALL_DIR}"

# 1. 安装依赖
echo "安装必要的依赖..."
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# 2. 下载 RealSense SDK 源码
echo "下载 RealSense SDK ${SDK_VERSION} 源码..."
mkdir -p "${SOURCE_DIR}"
cd "${SOURCE_DIR}"
if [ ! -d "librealsense" ]; then
    git clone https://github.com/IntelRealSense/librealsense.git
fi
cd librealsense
git checkout "${SDK_VERSION}"
# ./scripts/setup_udev_rules.sh


# 3. 配置 USB 权限
echo "配置 USB 权限..."
sudo rm -rf /etc/udev/rules.d/99-realsense-libusb.rules
sudo cp ./config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger || true



# 4. 配置编译环境
echo "配置 CMake..."
mkdir -p build && cd build
cmake .. -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true -DBUILD_TOOLS=true -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}/"

# 5. 编译源码默认为release版本
echo "编译 RealSense SDK..."
make -j$(nproc)

# 6. 安装 SDK
echo "安装 RealSense SDK 到 ${INSTALL_DIR}/..."
make install


# 7. 验证安装
echo "验证安装..."
if [ -f "${INSTALL_DIR}/bin/realsense-viewer" ]; then
    echo "安装成功！可运行以下命令测试："
    echo "${INSTALL_DIR}/bin/realsense-viewer"
else
    echo "安装失败，请检查日志！"
    exit 1
fi

# 8. 清除
echo "清除..."
cd ${SOURCE_DIR}
rm -rf librealsense

echo "RealSense SDK ${SDK_VERSION} 已成功安装到 ${INSTALL_DIR}"
