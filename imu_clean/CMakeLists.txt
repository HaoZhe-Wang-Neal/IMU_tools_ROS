cmake_minimum_required(VERSION 3.0.2)
project(imu_clean)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)


# 查找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  eigen_conversions
)
find_package(Eigen3 REQUIRED)


# 声明catkin包
catkin_package(
  INCLUDE_DIRS cpp/include
  LIBRARIES imu_clean_cpp
  CATKIN_DEPENDS roscpp rospy sensor_msgs
  DEPENDS EIGEN3
)



#C++
# 包含头文件
include_directories(
  cpp/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)



# 编译核心库
add_library(imu_clean_cpp
  cpp/src/imu_data_cleaner.cpp
)
target_link_libraries(imu_clean_cpp ${catkin_LIBRARIES})

# 编译C++节点
add_executable(imu_clean_node_cpp
  cpp/src/imu_clean_node.cpp
)
target_link_libraries(imu_clean_node_cpp
  imu_clean_cpp
  ${catkin_LIBRARIES}
)



# 安装C++节点
install(TARGETS imu_clean_node_cpp imu_clean_cpp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)



# 安装C++头文件
install(DIRECTORY cpp/include/imu_clean/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)




# 安装Python模块
catkin_python_setup()

# 安装Python节点脚本
install(DIRECTORY python/imu_clean/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# 安装Python可执行节点
install(PROGRAMS
  python/imu_clean/imu_clean_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  RENAME imu_clean_node_py
)