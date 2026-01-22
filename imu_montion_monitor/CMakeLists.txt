cmake_minimum_required(VERSION 3.0.2)
project(imu_motion_monitor)

# 设置编译选项
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-O3 -Wall")

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  yaml-cpp
)

# 生成自定义消息
add_message_files(
  FILES
  ImuAnomaly.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# 声明catkin包
catkin_package(
  INCLUDE_DIRS cpp/include
  LIBRARIES imu_motion_monitor
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime
  DEPENDS yaml-cpp
)

# 包含头文件目录
include_directories(
  cpp/include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# 编译C++核心库
add_library(imu_motion_monitor
  cpp/src/imu_motion_monitor.cpp
)

# 链接依赖
target_link_libraries(imu_motion_monitor
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# 编译ROS节点可执行文件
add_executable(imu_motion_monitor_node
  cpp/src/imu_motion_monitor_node.cpp
)

# 节点链接库
target_link_libraries(imu_motion_monitor_node
  imu_motion_monitor
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# 确保消息生成在节点编译前
add_dependencies(imu_motion_monitor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 安装C++可执行文件
install(TARGETS imu_motion_monitor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装Python脚本
catkin_install_python(PROGRAMS
  python/imu_motion_monitor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装配置文件和启动文件
install(DIRECTORY config/ launch/ msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装头文件
install(DIRECTORY cpp/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)