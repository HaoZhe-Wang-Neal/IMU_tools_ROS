#include "imu_motion_monitor/imu_motion_monitor.h"
#include <ros/console.h>
#include <fstream>

// 辅助函数：计算方差
double calculateVariance(const std::vector<double>& data) {
  if (data.empty()) return 0.0;

  // 计算均值
  double sum = 0.0;
  for (double val : data) sum += val;
  double mean = sum / data.size();

  // 计算方差
  double var_sum = 0.0;
  for (double val : data) {
    var_sum += (val - mean) * (val - mean);
  }
  return var_sum / data.size();
}

// 构造函数
ImuMotionMonitor::ImuMotionMonitor(ros::NodeHandle& nh, const std::string& config_path)
  : nh_(nh) {
  // 1. 加载配置
  config_ = loadConfig(config_path);
  ROS_INFO("[IMU Motion Monitor] Loaded config for scene: %s", config_.active_scene.c_str());

  // 2. 订阅IMU话题
  imu_sub_ = nh_.subscribe(config_.imu_topic, 100, &ImuMotionMonitor::imuCallback, this);
  ROS_INFO("[IMU Motion Monitor] Subscribed to IMU topic: %s", config_.imu_topic.c_str());

  // 3. 创建异常消息发布器
  if (config_.anomaly_msg_type == "bool") {
    anomaly_pub_ = nh_.advertise<std_msgs::Bool>(config_.anomaly_topic, 10);
  } else if (config_.anomaly_msg_type == "custom") {
    anomaly_pub_ = nh_.advertise<imu_motion_monitor::ImuAnomaly>(config_.anomaly_topic, 10);
  }
  ROS_INFO("[IMU Motion Monitor] Will publish anomaly to: %s (type: %s)", 
           config_.anomaly_topic.c_str(), config_.anomaly_msg_type.c_str());
}

// 加载配置文件
MonitorConfig ImuMotionMonitor::loadConfig(const std::string& config_path) {
  MonitorConfig config;

  // 读取YAML文件
  YAML::Node yaml_config;
  try {
    yaml_config = YAML::LoadFile(config_path);
  } catch (const YAML::BadFile& e) {
    ROS_ERROR("[IMU Motion Monitor] Failed to load config file: %s", e.what());
    ros::shutdown();
    return config;
  }

  // 1. 加载全局配置
  if (yaml_config["global"]) {
    config.imu_topic = yaml_config["global"]["imu_topic"].as<std::string>();
    config.window_size = yaml_config["global"]["window_size"].as<int>();
    config.anomaly_msg_type = yaml_config["global"]["anomaly_msg_type"].as<std::string>();
  }

  // 2. 获取激活场景
  config.active_scene = yaml_config["active_scene"].as<std::string>();
  YAML::Node scene_config = yaml_config[config.active_scene];
  if (!scene_config) {
    ROS_ERROR("[IMU Motion Monitor] Scene %s not found in config file!", config.active_scene.c_str());
    ros::shutdown();
    return config;
  }

  // 3. 加载场景配置（覆盖全局同名配置）
  if (scene_config["imu_topic"]) {
    config.imu_topic = scene_config["imu_topic"].as<std::string>();
  }
  if (scene_config["window_size"]) {
    config.window_size = scene_config["window_size"].as<int>();
  }
  config.anomaly_topic = scene_config["anomaly_topic"].as<std::string>();
  config.anomaly_type = scene_config["anomaly_type"].as<std::string>();

  // 4. 加载场景检测阈值
  YAML::Node thresholds = scene_config["detect_thresholds"];
  for (YAML::const_iterator it = thresholds.begin(); it != thresholds.end(); ++it) {
    config.detect_thresholds[it->first.as<std::string>()] = it->second.as<double>();
  }

  return config;
}

// IMU数据回调
void ImuMotionMonitor::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  // 1. 缓存IMU数据（仅核心字段）
  ImuData data;
  data.angular_vel_x = msg->angular_velocity.x;
  data.angular_vel_y = msg->angular_velocity.y;
  data.angular_vel_z = msg->angular_velocity.z;
  data.linear_acc_x = msg->linear_acceleration.x;
  data.linear_acc_y = msg->linear_acceleration.y;
  data.linear_acc_z = msg->linear_acceleration.z;
  data.stamp = msg->header.stamp;

  imu_window_.push_back(data);

  // 2. 维护滑动窗口大小
  if (imu_window_.size() > config_.window_size) {
    imu_window_.pop_front();
  }

  // 3. 窗口填满后才进行检测
  if (imu_window_.size() == config_.window_size) {
    // 提取特征
    std::map<std::string, double> features = extractFeatures(imu_window_);
    
    // 异常检测
    bool is_anomaly = detectAnomaly(features);
    
    // 发布异常消息（仅异常时发布，避免无效通信）
    if (is_anomaly) {
      publishAnomaly(is_anomaly);
      ROS_INFO("[%s] Detected anomaly: %s", config_.active_scene.c_str(), config_.anomaly_type.c_str());
    }
  }
}

// 特征提取（场景化）
std::map<std::string, double> ImuMotionMonitor::extractFeatures(const std::deque<ImuData>& window) {
  std::map<std::string, double> features;

  if (config_.active_scene == "mobile_robot") {
    // 场景1：移动机器人-轮子打滑
    // 特征1：z轴角速度帧间差值（当前帧 - 窗口首帧）
    double angular_z_diff = fabs(window.back().angular_vel_z - window.front().angular_vel_z);
    features["angular_z_diff"] = angular_z_diff;

    // 特征2：x/y轴加速度方差
    std::vector<double> accel_xy;
    for (const auto& data : window) {
      double accel_xy_mag = sqrt(data.linear_acc_x*data.linear_acc_x + data.linear_acc_y*data.linear_acc_y);
      accel_xy.push_back(accel_xy_mag);
    }
    features["accel_xy_var"] = calculateVariance(accel_xy);

  } else if (config_.active_scene == "drone") {
    // 场景2：无人机-机身抖震
    // 特征：三轴角速度方差均值
    std::vector<double> ang_x, ang_y, ang_z;
    for (const auto& data : window) {
      ang_x.push_back(data.angular_vel_x);
      ang_y.push_back(data.angular_vel_y);
      ang_z.push_back(data.angular_vel_z);
    }
    double var_x = calculateVariance(ang_x);
    double var_y = calculateVariance(ang_y);
    double var_z = calculateVariance(ang_z);
    features["angular_var_avg"] = (var_x + var_y + var_z) / 3.0;

  } else if (config_.active_scene == "arm") {
    // 场景3：机械臂-碰撞冲击
    // 特征：三轴加速度绝对值最大值
    double max_accel = 0.0;
    for (const auto& data : window) {
      double acc_x_abs = fabs(data.linear_acc_x);
      double acc_y_abs = fabs(data.linear_acc_y);
      double acc_z_abs = fabs(data.linear_acc_z);
      max_accel = std::max({max_accel, acc_x_abs, acc_y_abs, acc_z_abs});
    }
    features["accel_peak"] = max_accel;
  }

  return features;
}

// 异常检测（场景化）
bool ImuMotionMonitor::detectAnomaly(const std::map<std::string, double>& features) {
  bool is_anomaly = false;

  if (config_.active_scene == "mobile_robot") {
    // 规则：|z轴角速度差值| > 阈值 AND x/y加速度方差 < 阈值
    double ang_z_diff = features.at("angular_z_diff");
    double accel_xy_var = features.at("accel_xy_var");
    double th_ang_z = config_.detect_thresholds["angular_z_diff"];
    double th_accel_xy = config_.detect_thresholds["accel_xy_var"];
    
    is_anomaly = (ang_z_diff > th_ang_z) && (accel_xy_var < th_accel_xy);

  } else if (config_.active_scene == "drone") {
    // 规则：三轴角速度方差均值 > 阈值
    double ang_var_avg = features.at("angular_var_avg");
    double th_ang_var = config_.detect_thresholds["angular_var_avg"];
    
    is_anomaly = (ang_var_avg > th_ang_var);

  } else if (config_.active_scene == "arm") {
    // 规则：加速度峰值 > 阈值
    double accel_peak = features.at("accel_peak");
    double th_accel_peak = config_.detect_thresholds["accel_peak"];
    
    is_anomaly = (accel_peak > th_accel_peak);
  }

  return is_anomaly;
}

// 发布异常消息
void ImuMotionMonitor::publishAnomaly(bool is_anomaly) {
  if (config_.anomaly_msg_type == "bool") {
    std_msgs::Bool msg;
    msg.data = is_anomaly;
    anomaly_pub_.publish(msg);
  } else if (config_.anomaly_msg_type == "custom") {
    imu_motion_monitor::ImuAnomaly msg;
    msg.stamp = ros::Time::now();
    msg.is_anomaly = is_anomaly;
    msg.anomaly_type = config_.anomaly_type;
    anomaly_pub_.publish(msg);
  }
}