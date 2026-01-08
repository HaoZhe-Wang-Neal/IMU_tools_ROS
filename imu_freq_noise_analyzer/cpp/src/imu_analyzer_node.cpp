#include "imu_freq_noise_analyzer\cpp\include\imu_freq_noise_analyzer\imu_analyzer_node.h"

// 静态成员初始化
OnlineImuSubscriber* ImuFreqNoiseAnalyzerNode::subscriber_ptr_ = nullptr;

// 信号处理回调
void ImuFreqNoiseAnalyzerNode::sigintHandler(int sig) {
  if (subscriber_ptr_) {
    subscriber_ptr_->stop();
    ROS_INFO("接收到退出信号，停止在线数据采集...");
  }
  ros::shutdown();
}

// 构造函数
ImuFreqNoiseAnalyzerNode::ImuFreqNoiseAnalyzerNode(int argc, char** argv) 
  : nh_("~"), argc_(argc), argv_(argv) {
  ros::init(argc, argv, "imu_freq_noise_analyzer_node", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigintHandler);
}

// 加载配置
void ImuFreqNoiseAnalyzerNode::loadConfig() {
  if (argc_ >= 2) {
    std::string config_path = argv_[1];
    config_ = loadConfig(config_path);
    ROS_INFO("从配置文件加载参数：%s", config_path.c_str());
  } else {
    config_ = loadRosParams(nh_);
    ROS_INFO("从ROS参数服务器加载参数");
  }
}

// 校验配置
bool ImuFreqNoiseAnalyzerNode::validateConfig() {
  if (config_.mode != "offline" && config_.mode != "online") {
    ROS_ERROR("无效模式：%s（仅支持offline/online）", config_.mode.c_str());
    return false;
  }
  if (config_.mode == "offline" && config_.bag_path.empty()) {
    ROS_ERROR("离线模式必须指定bag_path");
    return false;
  }
  return true;
}

// 采集IMU数据
std::vector<ImuData> ImuFreqNoiseAnalyzerNode::collectImuData() {
  std::vector<ImuData> imu_data;
  if (config_.mode == "offline") {
    imu_data = loadFromBag(config_);
  } else if (config_.mode == "online") {
    OnlineImuSubscriber subscriber(config_);
    subscriber_ptr_ = &subscriber;
    double collect_duration = config_.collect_duration;
    nh_.param<double>("imu_analyzer/collect_duration", collect_duration, collect_duration);
    imu_data = subscriber.getData(collect_duration);
    subscriber_ptr_ = nullptr;
  }
  return imu_data;
}

// 运行节点
int ImuFreqNoiseAnalyzerNode::run() {
  loadConfig();
  if (!validateConfig()) return -1;

  std::vector<ImuData> imu_data = collectImuData();
  if (imu_data.empty()) {
    ROS_ERROR("未获取到有效IMU数据");
    return -1;
  }

  if (config_.preprocess.enable) {
    filterOutliers(imu_data, config_.preprocess.sigma);
  }

  freqAnalysis(imu_data, config_);
  allanAnalysis(imu_data, config_);

  ROS_INFO("="*50);
  ROS_INFO("IMU频率/噪声分析完成！");
  ROS_INFO("结果保存目录：%s", config_.output.dir.c_str());
  ROS_INFO("="*50);

  return 0;
}

// 主函数入口
int main(int argc, char** argv) {
  try {
    ImuFreqNoiseAnalyzerNode node(argc, argv);
    return node.run();
  } catch (const std::exception& e) {
    ROS_ERROR("节点运行异常：%s", e.what());
    return -1;
  }
}