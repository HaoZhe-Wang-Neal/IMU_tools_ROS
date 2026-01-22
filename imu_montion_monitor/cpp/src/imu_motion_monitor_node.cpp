#include "imu_motion_monitor/imu_motion_monitor.h"
#include <ros/ros.h>
#include <string>

int main(int argc, char** argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "imu_motion_monitor_node");
  ros::NodeHandle nh("~");

  // 获取配置文件路径（从ROS参数或默认路径）
  std::string config_path;
  if (!nh.getParam("config_path", config_path)) {
    // 默认路径：功能包share目录下的config文件
    config_path = ros::package::getPath("imu_motion_monitor") + "/config/imu_motion_monitor.yaml";
  }

  // 创建异常检测实例
  ImuMotionMonitor monitor(nh, config_path);

  // 自旋
  ROS_INFO("[IMU Motion Monitor] Node started successfully!");
  ros::spin();

  return 0;
}