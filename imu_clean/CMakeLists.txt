cmake_minimum_required(VERSION 2.8.3)
project(imu_clean)

find_package(catkin REQUIRED COMPONENTS
  catkin
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure 
  Boost
  Eigen
)

generate_dynamic_reconfigure_options(
  cfg/ImuClean.cfg
)

add_compile_options(-pthread)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs dynamic_reconfigure
  DEPENDS Boost Eigen 
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

# C++节点编译
add_executable(${PROJECT_NAME}_node cpp/imu_clean_node.cpp)
# 链接Boost/Eigen/动态参数库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen_LIBRARIES}
)
#确保动态参数头文件生成后再编译
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
#安装C++可执行文件
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#安装Python脚本
install(PROGRAMS
  python/imu_clean_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装launch/config目录
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)