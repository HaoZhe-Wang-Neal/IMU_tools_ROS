cmake_minimum_required(VERSION 3.10)
project(imu_freq_noise_analyzer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# 查找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  rosbag
  Eigen3
)

find_package(YAML-CPP REQUIRED)
find_package(FFTW3 REQUIRED)

# Catkin配置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs rosbag
  DEPENDS EIGEN3 YAML-CPP FFTW3
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${FFTW3_INCLUDE_DIRS}
)

# 编译核心库
add_library(${PROJECT_NAME}
  src/imu_analyzer_core.cpp
)

# 链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${FFTW3_LIBRARIES}
)

# 编译C++节点
add_executable(${PROJECT_NAME}_node
  src/imu_analyzer_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 安装
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/ launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Python脚本安装
catkin_install_python(PROGRAMS
  scripts/imu_analyzer_node.py
  scripts/imu_analyzer_core.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Python模块
catkin_python_setup()