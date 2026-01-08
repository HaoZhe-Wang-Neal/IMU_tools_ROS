#include "imu_freq_noise_analyzer\cpp\include\imu_freq_noise_analyzer\imu_analyzer_core.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <algorithm>

// 工具函数：创建目录（支持多级）
bool createDir(const std::string& dir) {
  if (dir.empty()) return true;
  DIR* d = opendir(dir.c_str());
  if (d) { closedir(d); return true; }
  
  size_t pos = 0;
  std::string temp;
  while ((pos = dir.find('/', pos + 1)) != std::string::npos) {
    temp = dir.substr(0, pos);
    if (temp.empty()) continue;
    mkdir(temp.c_str(), 0755);
  }
  int ret = mkdir(dir.c_str(), 0755);
  return ret == 0 || errno == EEXIST;
}

// 从YAML加载配置
ImuAnalyzerConfig loadConfig(const std::string& config_path) {
  ImuAnalyzerConfig config;
  try {
    YAML::Node yaml = YAML::LoadFile(config_path);
    YAML::Node root = yaml["imu_analyzer"];
    
    // 基础配置
    if (root["mode"]) config.mode = root["mode"].as<std::string>();
    if (root["imu_topic"]) config.imu_topic = root["imu_topic"].as<std::string>();
    if (root["mag_topic"]) config.mag_topic = root["mag_topic"].as<std::string>();
    if (root["use_mag"]) config.use_mag = root["use_mag"].as<bool>();
    if (root["sample_rate"]) config.sample_rate = root["sample_rate"].as<double>();
    
    // 离线配置
    if (root["bag_path"]) config.bag_path = root["bag_path"].as<std::string>();
    if (root["start_time"]) config.start_time = root["start_time"].as<double>();
    if (root["end_time"]) config.end_time = root["end_time"].as<double>();
    
    // 预处理配置
    if (root["preprocess"]["enable"]) config.preprocess.enable = root["preprocess"]["enable"].as<bool>();
    if (root["preprocess"]["sigma"]) config.preprocess.sigma = root["preprocess"]["sigma"].as<double>();
    if (root["preprocess"]["lowpass_cutoff"]) config.preprocess.lowpass_cutoff = root["preprocess"]["lowpass_cutoff"].as<double>();
    if (root["preprocess"]["resample_rate"]) config.preprocess.resample_rate = root["preprocess"]["resample_rate"].as<double>();
    
    // 频域分析配置
    if (root["freq_analysis"]["enable"]) config.freq_analysis.enable = root["freq_analysis"]["enable"].as<bool>();
    if (root["freq_analysis"]["fft_size"]) config.freq_analysis.fft_size = root["freq_analysis"]["fft_size"].as<int>();
    if (root["freq_analysis"]["window"]) config.freq_analysis.window = root["freq_analysis"]["window"].as<std::string>();
    
    // Allan分析配置
    if (root["allan_analysis"]["enable"]) config.allan_analysis.enable = root["allan_analysis"]["enable"].as<bool>();
    if (root["allan_analysis"]["max_tau"]) config.allan_analysis.max_tau = root["allan_analysis"]["max_tau"].as<double>();
    if (root["allan_analysis"]["tau_num"]) config.allan_analysis.tau_num = root["allan_analysis"]["tau_num"].as<int>();
    
    // 输出配置
    if (root["output"]["dir"]) config.output.dir = root["output"]["dir"].as<std::string>();
    if (root["output"]["plot"]) config.output.plot = root["output"]["plot"].as<bool>();
    
    // 在线扩展配置
    if (root["collect_duration"]) config.collect_duration = root["collect_duration"].as<double>();
  } catch (YAML::Exception& e) {
    ROS_ERROR("加载YAML配置失败: %s", e.what());
  }
  return config;
}

// 从ROS参数服务器加载配置
ImuAnalyzerConfig loadRosParams(ros::NodeHandle& nh) {
  ImuAnalyzerConfig config;
  
  // 基础配置
  nh.param<std::string>("imu_analyzer/mode", config.mode, "offline");
  nh.param<std::string>("imu_analyzer/imu_topic", config.imu_topic, "/imu/data_raw");
  nh.param<std::string>("imu_analyzer/mag_topic", config.mag_topic, "/imu/mag");
  nh.param<bool>("imu_analyzer/use_mag", config.use_mag, false);
  nh.param<double>("imu_analyzer/sample_rate", config.sample_rate, 100.0);
  
  // 离线配置
  nh.param<std::string>("imu_analyzer/bag_path", config.bag_path, "");
  nh.param<double>("imu_analyzer/start_time", config.start_time, 0.0);
  nh.param<double>("imu_analyzer/end_time", config.end_time, 0.0);
  
  // 预处理配置
  nh.param<bool>("imu_analyzer/preprocess/enable", config.preprocess.enable, false);
  nh.param<double>("imu_analyzer/preprocess/sigma", config.preprocess.sigma, 3.0);
  nh.param<double>("imu_analyzer/preprocess/lowpass_cutoff", config.preprocess.lowpass_cutoff, 10.0);
  nh.param<double>("imu_analyzer/preprocess/resample_rate", config.preprocess.resample_rate, 100.0);
  
  // 频域分析配置
  nh.param<bool>("imu_analyzer/freq_analysis/enable", config.freq_analysis.enable, true);
  nh.param<int>("imu_analyzer/freq_analysis/fft_size", config.freq_analysis.fft_size, 1024);
  nh.param<std::string>("imu_analyzer/freq_analysis/window", config.freq_analysis.window, "hann");
  
  // Allan分析配置
  nh.param<bool>("imu_analyzer/allan_analysis/enable", config.allan_analysis.enable, true);
  nh.param<double>("imu_analyzer/allan_analysis/max_tau", config.allan_analysis.max_tau, 100.0);
  nh.param<int>("imu_analyzer/allan_analysis/tau_num", config.allan_analysis.tau_num, 100);
  
  // 输出配置
  nh.param<std::string>("imu_analyzer/output/dir", config.output.dir, "./imu_analysis_results");
  nh.param<bool>("imu_analyzer/output/plot", config.output.plot, true);
  
  // 在线扩展配置
  nh.param<double>("imu_analyzer/collect_duration", config.collect_duration, 0.0);
  
  return config;
}

// 离线从bag加载数据
std::vector<ImuData> loadFromBag(const ImuAnalyzerConfig& config) {
  std::vector<ImuData> imu_data_list;
  if (config.bag_path.empty()) {
    ROS_ERROR("离线模式必须指定bag_path");
    return imu_data_list;
  }

  rosbag::Bag bag;
  try {
    bag.open(config.bag_path, rosbag::bagmode::Read);
  } catch (std::exception& e) {
    ROS_ERROR("打开Bag失败: %s (错误: %s)", config.bag_path.c_str(), e.what());
    return imu_data_list;
  }

  ros::Time start_time(0), end_time(0);
  if (config.start_time > 0) start_time = ros::Time(config.start_time);
  if (config.end_time > 0) end_time = ros::Time(config.end_time);

  std::vector<std::string> topics;
  topics.push_back(config.imu_topic);
  if (config.use_mag) topics.push_back(config.mag_topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics), start_time, end_time);
  std::vector<ImuData> mag_data_list;

  for (const auto& m : view) {
    if (m.getTopic() == config.imu_topic) {
      sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
      if (!imu_msg) continue;
      ImuData data;
      data.timestamp = m.getTime().toSec();
      data.accel << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z;
      data.gyro << imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
      imu_data_list.push_back(data);
    }
    if (config.use_mag && m.getTopic() == config.mag_topic) {
      sensor_msgs::MagneticField::ConstPtr mag_msg = m.instantiate<sensor_msgs::MagneticField>();
      if (!mag_msg) continue;
      ImuData data;
      data.timestamp = m.getTime().toSec();
      data.mag << mag_msg->magnetic_field.x, mag_msg->magnetic_field.y, mag_msg->magnetic_field.z;
      data.has_mag = true;
      mag_data_list.push_back(data);
    }
  }

  if (config.use_mag && !mag_data_list.empty()) {
    for (auto& imu_data : imu_data_list) {
      auto it = std::min_element(mag_data_list.begin(), mag_data_list.end(),
        [&](const ImuData& a, const ImuData& b) {
          return fabs(a.timestamp - imu_data.timestamp) < fabs(b.timestamp - imu_data.timestamp);
        });
      if (it != mag_data_list.end() && fabs(it->timestamp - imu_data.timestamp) < 0.01) {
        imu_data.mag = it->mag;
        imu_data.has_mag = true;
      }
    }
  }

  bag.close();
  ROS_INFO("离线加载IMU数据: %lu个点", imu_data_list.size());
  return imu_data_list;
}

// 在线订阅类实现
OnlineImuSubscriber::OnlineImuSubscriber(const ImuAnalyzerConfig& config) : config_(config) {
  imu_sub_ = nh_.subscribe(config.imu_topic, 1000, &OnlineImuSubscriber::imuCallback, this);
  if (config.use_mag) {
    mag_sub_ = nh_.subscribe(config.mag_topic, 1000, &OnlineImuSubscriber::magCallback, this);
  }
  ROS_INFO("在线模式：订阅IMU话题 %s", config.imu_topic.c_str());
  if (config.use_mag) {
    ROS_INFO("在线模式：订阅磁力计话题 %s", config.mag_topic.c_str());
  }
}

void OnlineImuSubscriber::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  if (!is_running_) return;
  ImuData data;
  data.timestamp = msg->header.stamp.toSec();
  data.accel << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  data.gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  imu_data_list_.push_back(data);
}

void OnlineImuSubscriber::magCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
  if (!is_running_) return;
  ImuData data;
  data.timestamp = msg->header.stamp.toSec();
  data.mag << msg->magnetic_field.x, msg->magnetic_field.y, msg->magnetic_field.z;
  data.has_mag = true;
  mag_data_list_.push_back(data);
}

std::vector<ImuData> OnlineImuSubscriber::getData(double duration) {
  is_running_ = true;
  imu_data_list_.clear();
  mag_data_list_.clear();

  if (duration > 0) {
    ROS_INFO("在线采集数据中...（时长：%.2fs）", duration);
    ros::Rate rate(100);
    ros::Time start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start).toSec() < duration) {
      ros::spinOnce();
      rate.sleep();
    }
  } else {
    ROS_INFO("在线采集数据中...（按Ctrl+C停止）");
    ros::Rate rate(10);
    while (ros::ok() && is_running_) {
      ros::spinOnce();
      rate.sleep();
    }
  }

  is_running_ = false;
  if (config_.use_mag && !mag_data_list_.empty()) {
    for (auto& imu_data : imu_data_list_) {
      auto it = std::min_element(mag_data_list_.begin(), mag_data_list_.end(),
        [&](const ImuData& a, const ImuData& b) {
          return fabs(a.timestamp - imu_data.timestamp) < fabs(b.timestamp - imu_data.timestamp);
        });
      if (it != mag_data_list_.end() && fabs(it->timestamp - imu_data.timestamp) < 0.01) {
        imu_data.mag = it->mag;
        imu_data.has_mag = true;
      }
    }
  }

  ROS_INFO("在线采集完成：%lu个IMU数据点", imu_data_list_.size());
  return imu_data_list_;
}

void OnlineImuSubscriber::stop() {
  is_running_ = false;
}

// 3σ异常值过滤
void filterOutliers(std::vector<ImuData>& data, double sigma) {
  if (data.empty()) return;

  Eigen::MatrixXd accel_mat(data.size(), 3);
  Eigen::MatrixXd gyro_mat(data.size(), 3);
  for (int i = 0; i < data.size(); i++) {
    accel_mat.row(i) = data[i].accel;
    gyro_mat.row(i) = data[i].gyro;
  }

  Eigen::Vector3d accel_mean = accel_mat.colwise().mean();
  Eigen::Vector3d accel_std = ((accel_mat.rowwise() - accel_mean).array().square().colwise().mean()).sqrt();
  Eigen::Vector3d gyro_mean = gyro_mat.colwise().mean();
  Eigen::Vector3d gyro_std = ((gyro_mat.rowwise() - gyro_mean).array().square().colwise().mean()).sqrt();

  std::vector<ImuData> filtered;
  for (const auto& d : data) {
    bool accel_ok = ((d.accel - accel_mean).cwiseAbs() < sigma * accel_std).all();
    bool gyro_ok = ((d.gyro - gyro_mean).cwiseAbs() < sigma * gyro_std).all();
    if (accel_ok && gyro_ok) filtered.push_back(d);
  }

  int removed = data.size() - filtered.size();
  data = filtered;
  ROS_INFO("异常值过滤：移除%d个点，剩余%d个点", removed, data.size());
}

// 频域分析（PSD）
void freqAnalysis(const std::vector<ImuData>& data, const ImuAnalyzerConfig& config) {
  if (!config.freq_analysis.enable || data.empty()) return;

  int n = config.freq_analysis.fft_size;
  double fs = config.preprocess.resample_rate;
  createDir(config.output.dir);

  const std::vector<std::string> axes = {"x", "y", "z"};
  const std::vector<std::string> types = {"gyro", "accel"};

  for (const auto& type : types) {
    for (int ax = 0; ax < 3; ax++) {
      std::vector<double> values;
      for (const auto& d : data) {
        values.push_back(type == "gyro" ? d.gyro(ax) : d.accel(ax));
      }

      while (values.size() < n) values.push_back(0);
      values.resize(n);

      if (config.freq_analysis.window == "hann") {
        for (int i = 0; i < n; i++) {
          values[i] *= 0.5 * (1 - cos(2 * M_PI * i / (n - 1)));
        }
      }

      fftw_complex* in = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * n);
      fftw_complex* out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * n);
      fftw_plan plan = fftw_plan_dft_1d(n, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

      for (int i = 0; i < n; i++) {
        in[i][0] = values[i];
        in[i][1] = 0;
      }
      fftw_execute(plan);

      std::vector<double> psd(n/2 + 1);
      for (int i = 0; i <= n/2; i++) {
        double re = out[i][0];
        double im = out[i][1];
        psd[i] = (re*re + im*im) / (fs * n);
      }

      std::string filename = config.output.dir + "/" + type + "_" + axes[ax] + "_psd.csv";
      std::ofstream f(filename);
      f << "freq,psd\n";
      for (int i = 0; i <= n/2; i++) {
        double freq = i * fs / n;
        f << freq << "," << psd[i] << "\n";
      }
      f.close();

      fftw_destroy_plan(plan);
      fftw_free(in);
      fftw_free(out);
    }
  }

  ROS_INFO("频域分析完成：PSD文件已保存到 %s", config.output.dir.c_str());
}

// Allan方差分析
void allanAnalysis(const std::vector<ImuData>& data, const ImuAnalyzerConfig& config) {
  if (!config.allan_analysis.enable || data.empty()) return;

  double dt = 1.0 / config.preprocess.resample_rate;
  int tau_num = config.allan_analysis.tau_num;
  double max_tau = config.allan_analysis.max_tau;
  createDir(config.output.dir);

  std::vector<double> taus;
  double tau_min = dt;
  double step = (log10(max_tau) - log10(tau_min)) / (tau_num - 1);
  for (int i = 0; i < tau_num; i++) {
    taus.push_back(pow(10, log10(tau_min) + i * step));
  }

  const std::vector<std::string> axes = {"x", "y", "z"};
  const std::vector<std::string> types = {"gyro", "accel"};

  for (const auto& type : types) {
    for (int ax = 0; ax < 3; ax++) {
      std::vector<double> values;
      for (const auto& d : data) {
        values.push_back(type == "gyro" ? d.gyro(ax) : d.accel(ax));
      }

      std::vector<double> allan_var(tau_num, 0);
      for (int i = 0; i < tau_num; i++) {
        int m = round(taus[i] / dt);
        if (m < 1 || m >= values.size()) continue;

        int n = values.size() - 2*m;
        double var = 0;
        for (int k = 0; k < n; k++) {
          double diff = 0;
          for (int j = 0; j < m; j++) {
            diff += values[k + m + j] - values[k + j];
          }
          var += diff * diff;
        }
        allan_var[i] = var / (2 * m * m * n);
      }

      std::string filename = config.output.dir + "/" + type + "_" + axes[ax] + "_allan.csv";
      std::ofstream f(filename);
      f << "tau,allan_var,allan_deviation\n";
      for (int i = 0; i < tau_num; i++) {
        double dev = sqrt(allan_var[i]);
        f << taus[i] << "," << allan_var[i] << "," << dev << "\n";
      }
      f.close();
    }
  }

  ROS_INFO("Allan方差分析完成：结果已保存到 %s", config.output.dir.c_str());
}