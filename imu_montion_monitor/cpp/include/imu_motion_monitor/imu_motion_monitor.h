#ifndef IMU_MOTION_MONITOR_H
#define IMU_MOTION_MONITOR_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <imu_motion_monitor/ImuAnomaly.h>
#include <yaml-cpp/yaml.h>
#include <deque>
#include <string>
#include <vector>
#include <cmath>

// 配置结构体
struct MonitorConfig {
  // 全局配置（可被场景覆盖）
  std::string imu_topic = "/imu/data";
  int window_size = 5;
  std::string anomaly_msg_type = "bool";

  // 场景专属配置
  std::string active_scene;
  std::string anomaly_topic;
  std::string anomaly_type;
  
  // 场景阈值（不同场景有不同阈值，用map存储）
  std::map<std::string, double> detect_thresholds;
};

// IMU数据缓存结构体
struct ImuData {
  double angular_vel_x;
  double angular_vel_y;
  double angular_vel_z;
  double linear_acc_x;
  double linear_acc_y;
  double linear_acc_z;
  ros::Time stamp;
};

// 核心异常检测类
class ImuMotionMonitor {
public:
  /**
   * @brief 构造函数
   * @param nh ROS节点句柄
   * @param config_path 配置文件路径
   */
  ImuMotionMonitor(ros::NodeHandle& nh, const std::string& config_path);

  /**
   * @brief IMU数据回调函数
   * @param msg IMU原始数据消息
   */
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

private:
  /**
   * @brief 加载配置文件
   * @param config_path 配置文件路径
   * @return 合并后的有效配置
   */
  MonitorConfig loadConfig(const std::string& config_path);

  /**
   * @brief 特征提取（场景化）
   * @param window IMU数据滑动窗口
   * @return 提取的特征值（map键为特征名，值为特征值）
   */
  std::map<std::string, double> extractFeatures(const std::deque<ImuData>& window);

  /**
   * @brief 异常检测（场景化）
   * @param features 提取的特征值
   * @return 是否检测到异常
   */
  bool detectAnomaly(const std::map<std::string, double>& features);

  /**
   * @brief 发布异常消息（支持bool/custom两种类型）
   * @param is_anomaly 是否异常
   */
  void publishAnomaly(bool is_anomaly);

  // ROS相关
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_;
  ros::Publisher anomaly_pub_;

  // 配置
  MonitorConfig config_;

  // 数据缓存
  std::deque<ImuData> imu_window_;
};

// 辅助函数：计算方差
double calculateVariance(const std::vector<double>& data);

#endif // IMU_MOTION_MONITOR_H