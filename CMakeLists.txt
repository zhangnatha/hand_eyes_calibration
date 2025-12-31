cmake_minimum_required(VERSION 3.10)
project(Pose6D)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIRDPARTY_DIR "${CMAKE_SOURCE_DIR}/3rdparty")
message(STATUS "THIRDPARTY_DIR: ${THIRDPARTY_DIR}")

set(OpenCV_DIR "${THIRDPARTY_DIR}/opencv/lib/cmake/opencv4")
if (NOT EXISTS ${OpenCV_DIR})
    message(WARNING "OpenCV path does not exist: ${OpenCV_DIR}. Trying system OpenCV...")
    find_package(OpenCV REQUIRED)
else()
    find_package(OpenCV REQUIRED PATHS ${OpenCV_DIR})
endif()

# 查找 Eigen
set(EIGEN3_DIR "${THIRDPARTY_DIR}/eigen/share/eigen3/cmake")
find_path(EIGEN3_INCLUDE_DIR Eigen/Core HINTS "${THIRDPARTY_DIR}/eigen/include/eigen3")
message(STATUS "EIGEN3_DIR: ${EIGEN3_DIR}")
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
if (NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Eigen 3.4.0 not found. Please install Eigen in ${THIRDPARTY_DIR}/eigen.")
endif()

find_library(REALSENSE2_LIBRARY NAMES realsense2 HINTS "${THIRDPARTY_DIR}/camera_realsense/lib")
find_path(REALSENSE2_INCLUDE_DIR librealsense2/rs.hpp HINTS "${THIRDPARTY_DIR}/camera_realsense/include")
message(STATUS "REALSENSE2_LIBRARY: ${REALSENSE2_LIBRARY}")
message(STATUS "REALSENSE2_INCLUDE_DIR: ${REALSENSE2_INCLUDE_DIR}")
if (NOT REALSENSE2_LIBRARY OR NOT REALSENSE2_INCLUDE_DIR)
    message(FATAL_ERROR "librealsense2 not found. Please install librealsense2 v2.54.2 in ${THIRDPARTY_DIR}/camera_realsense.")
endif()

include_directories(
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${REALSENSE2_INCLUDE_DIR}
)

add_library(common STATIC
        src/common/Math.cpp
        src/common/CoordinateTransformer.cpp
        src/common/ArucoMarkerDetector.cpp
        src/calibration/01_intrinsic/IntrinsicCalibrator.cpp
        src/calibration/01_intrinsic/ChessBoardCalibrator.cpp
        src/calibration/02_hand_eye_2d_4point/FourPointCalibrator.cpp
        src/calibration/03_hand_eye_2d_12point/TwelvePointCalibrator.cpp
        src/calibration/04_hand_eye_3d_ball/BallCalibrator.cpp
        src/calibration/05_hand_eye_3d_board_img/BoardImageCalibrator.cpp
        src/calibration/06_hand_eye_3d_board_cloud/BoardCloudCalibrator.cpp
        src/calibration/hand_eye_3d_base/RobotCameraCalibrator3D.cpp
)

add_executable(calib calib.cpp)
target_link_libraries(calib common ${OpenCV_LIBS} ${REALSENSE2_LIBRARY})

add_executable(detect detect.cpp)
target_link_libraries(detect common ${OpenCV_LIBS} ${REALSENSE2_LIBRARY})