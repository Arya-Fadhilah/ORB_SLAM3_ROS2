cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
)

link_directories(
  include
)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp nav_msgs sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

add_executable(kitti_file_debug
  src/mono_kitti/kitti_file_debug.cpp
  src/mono_kitti/kitti-file-slam-node.cpp
)
ament_target_dependencies(kitti_file_debug rclcpp nav_msgs sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

add_executable(mono_inertial
  src/monocular-inertial/mono_inertial.cpp
  src/monocular-inertial/monocular-inertial-node.cpp
)
ament_target_dependencies(mono_inertial rclcpp nav_msgs sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

add_executable(mono_inertial_file_debug
  src/mono_imu_euroc_tum/mono_inertial_file_debug.cpp
  src/mono_imu_euroc_tum/mono-inertial-file-slam-node.cpp
)
ament_target_dependencies(mono_inertial_file_debug rclcpp nav_msgs sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp nav_msgs sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp nav_msgs sensor_msgs cv_bridge message_filters OpenCV
ORB_SLAM3 Pangolin)

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial nav_msgs rclcpp sensor_msgs cv_bridge OpenCV ORB_SLAM3 Pangolin)

install(TARGETS mono rgbd stereo stereo-inertial kitti_file_debug mono_inertial mono_inertial_file_debug
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()

