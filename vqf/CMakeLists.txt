cmake_minimum_required(VERSION 3.0.2)
project(vqf)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  eigen_catkin
)

## Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## Catkin config
catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs
  DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  cpp/src
)

## C++节点编译
add_executable(vqf_node cpp/src/vqf_node.cpp)
target_link_libraries(vqf_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

## Python模块安装
catkin_install_python(PROGRAMS
  python/vqf/vqf_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## 安装launch和config文件
install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)