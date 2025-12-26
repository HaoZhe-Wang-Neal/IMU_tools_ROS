cmake_minimum_required(VERSION 3.0.2)
project(bag2csv)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  rosbag           # 核心：bag文件处理
  geometry_msgs    # 处理Vector3/Quaternion等消息
)

# 声明catkin包
catkin_package(
  INCLUDE_DIRS cpp/include
  LIBRARIES bag2csv_cpp
  CATKIN_DEPENDS roscpp rospy sensor_msgs rosbag geometry_msgs
)

# C++编译配置
include_directories(
  cpp/include
  ${catkin_INCLUDE_DIRS}
)

# 编译C++核心库
add_executable(bag2csv_node_cpp
  cpp/src/bag2csv_node.cpp
)
target_link_libraries(bag2csv_node_cpp
  ${catkin_LIBRARIES}
)


# 安装C++节点
install(TARGETS bag2csv_node_cpp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装C++头文件
install(DIRECTORY cpp/include/bag2csv/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Python配置
catkin_python_setup()

# 安装Python模块
install(DIRECTORY python/bag2csv/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)



# 安装Python可执行节点
install(PROGRAMS
  python/bag2csv/bag2csv_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  RENAME bag2csv_node_py
)