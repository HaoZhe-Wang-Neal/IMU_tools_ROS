cmake_minimum_required(VERSION 2.8.3)
project(bag2csv)

find_package(catkin REQUIRED COMPONENTS
  catkin
  roscpp
  rospy
  rosbag
  sensor_msgs
  std_msgs
)

# Python模块配置，支持rosrun调用Python脚本
catkin_python_setup()

# 导出依赖，供其他包调用时解析
catkin_package(
  CATKIN_DEPENDS roscpp rospy rosbag sensor_msgs std_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# C++节点编译
add_executable(${PROJECT_NAME}_node cpp/bag2csv_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
# 修复：安装C++可执行文件，确保rosrun能找到
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 修复：安装Python脚本，确保rosrun能调用
install(PROGRAMS
  python/bag2csv_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 修复：安装launch/config目录，确保roslaunch能找到文件
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)