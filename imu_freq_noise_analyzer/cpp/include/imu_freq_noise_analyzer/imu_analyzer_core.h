#ifndef IMU_FREQ_NOISE_ANALYZER_IMU_ANALYZER_CORE_H
#define IMU_FREQ_NOISE_ANALYZER_IMU_ANALYZER_CORE_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <Eigen/Dense>
#include <fftw3.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <map>
#include <dirent.h>

// IMU数据结构体
struct ImuData {
  double timestamp;
  Eigen::Vector3d accel;
  Eigen::Vector3d gyro;
  Eigen::Vector3d mag;
  bool has_mag = false;
};

// 配置结构体（与YAML严格对应）
struct ImuAnalyzerConfig {
  // 基础配置
  std::string mode;          
  std::string imu_topic;     
  std::string mag_topic;     
  bool use_mag = false;      
  double sample_rate = 100.0;
  
  // 离线配置
  std::string bag_path;      
  double start_time = 0.0;   
  double end_time = 0.0;     
  
  // 预处理配置
  struct Preprocess {
    bool enable = false;     
    double sigma = 3.0;      
    double lowpass_cutoff = 10.0;
    double resample_rate = 100.0;
  } preprocess;

  // 频域分析配置
  struct FreqAnalysis {
    bool enable = true;      
    int fft_size = 1024;     
    std::string window = "hann";
  } freq_analysis;

  // Allan方差分析配置
  struct AllanAnalysis {
    bool enable = true;      
    double max_tau = 100.0;  
    int tau_num = 100;       
  } allan_analysis;

  // 输出配置
  struct Output {
    std::string dir = "./imu_analysis_results";
    bool plot = true;        
  } output;

  // 在线模式扩展配置
  double collect_duration = 0.0;
};

// 在线订阅类
class OnlineImuSubscriber {
public:
  OnlineImuSubscriber(const ImuAnalyzerConfig& config);
  std::vector<ImuData> getData(double duration = 0.0);
  void stop();                                        

private:
  ImuAnalyzerConfig config_;
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_;
  ros::Subscriber mag_sub_;
  std::vector<ImuData> imu_data_list_;
  std::vector<ImuData> mag_data_list_;
  bool is_running_ = true;

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg);
};

// 工具函数
bool createDir(const std::string& dir);
ImuAnalyzerConfig loadConfig(const std::string& config_path);
ImuAnalyzerConfig loadRosParams(ros::NodeHandle& nh);

// 核心算法函数
std::vector<ImuData> loadFromBag(const ImuAnalyzerConfig& config);
void filterOutliers(std::vector<ImuData>& data, double sigma);
void freqAnalysis(const std::vector<ImuData>& data, const ImuAnalyzerConfig& config);
void allanAnalysis(const std::vector<ImuData>& data, const ImuAnalyzerConfig& config);

#endif // IMU_FREQ_NOISE_ANALYZER_IMU_ANALYZER_CORE_H