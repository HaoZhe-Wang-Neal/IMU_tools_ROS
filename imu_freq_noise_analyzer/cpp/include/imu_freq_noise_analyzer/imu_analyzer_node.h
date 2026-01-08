#ifndef IMU_FREQ_NOISE_ANALYZER_IMU_ANALYZER_NODE_H
#define IMU_FREQ_NOISE_ANALYZER_IMU_ANALYZER_NODE_H

#include "imu_freq_noise_analyzer\cpp\include\imu_freq_noise_analyzer\imu_analyzer_core.h"
#include <ros/ros.h>
#include <signal.h>

/**
 * @brief IMU频率/噪声分析节点类
 * 封装ROS节点生命周期、配置加载、数据采集/分析、信号处理等核心逻辑
 */
class ImuFreqNoiseAnalyzerNode {
public:
  ImuFreqNoiseAnalyzerNode(int argc, char** argv);
  ~ImuFreqNoiseAnalyzerNode() = default;
  int run();

private:
  void loadConfig();
  bool validateConfig();
  std::vector<ImuData> collectImuData();
  static void sigintHandler(int sig);

  ros::NodeHandle nh_;
  ImuAnalyzerConfig config_;
  int argc_;
  char** argv_;
  static OnlineImuSubscriber* subscriber_ptr_;
};

#endif // IMU_FREQ_NOISE_ANALYZER_IMU_ANALYZER_NODE_H