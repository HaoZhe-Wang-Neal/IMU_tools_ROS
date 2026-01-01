#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>
#include <imu_clean/ImuCleanConfig.h> 
#include <boost/thread/mutex.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <mutex> 

// 全局参数与数据结构
struct ImuCleanParams {
  int window_size = 10;
  double std_threshold = 0.1;
  bool gyro_to_deg = false;
};

struct ImuData {
  double timestamp;
  Eigen::Vector3d accel; // 加速度
  Eigen::Vector3d gyro;  // 陀螺仪
};

class ImuCleaner {
private:
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_;
  ros::Publisher imu_pub_;
  dynamic_reconfigure::Server<imu_clean::ImuCleanConfig> dyn_srv_;
  ImuCleanParams params_;
  std::vector<ImuData> imu_window_;
  std::mutex mutex_; 

  // 动态参数回调函数
  void reconfigCallback(imu_clean::ImuCleanConfig& config, uint32_t level) {
    std::lock_guard<std::mutex> lock(mutex_); 
    params_.window_size = config.window_size;
    params_.std_threshold = config.std_threshold;
    params_.gyro_to_deg = config.gyro_to_deg;
    ROS_INFO("Dynamic params updated: window_size=%d, std_threshold=%.2f, gyro_to_deg=%d",
             params_.window_size, params_.std_threshold, params_.gyro_to_deg);
  }

  // 滑动窗口统计计算（均值+标准差）
  bool calculateStats(const std::vector<ImuData>& window, 
                      Eigen::Vector3d& accel_mean, Eigen::Vector3d& accel_std,
                      Eigen::Vector3d& gyro_mean, Eigen::Vector3d& gyro_std) {
  

    if (window.size() < params_.window_size) {
      ROS_WARN_THROTTLE(1, "Window not full (current: %lu, required: %d)", window.size(), params_.window_size);
      return false;
    }

    // 计算均值
    accel_mean.setZero();
    gyro_mean.setZero();
    for (const auto& data : window) {
      accel_mean += data.accel;
      gyro_mean += data.gyro;
    }
    accel_mean /= window.size();
    gyro_mean /= window.size();

    // 计算标准差
    accel_std.setZero();
    gyro_std.setZero();
    for (const auto& data : window) {
      accel_std += (data.accel - accel_mean).cwiseAbs2();
      gyro_std += (data.gyro - gyro_mean).cwiseAbs2();
    }
    accel_std = (accel_std / window.size()).cwiseSqrt();
    gyro_std = (gyro_std / window.size()).cwiseSqrt();

    return true;
  }

  // IMU回调函数
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    std::lock_guard<std::mutex> lock(mutex_); 

    // 过滤无效数据
    double timestamp = imu_msg->header.stamp.toSec();
    if (timestamp <= 0) {
      ROS_WARN_THROTTLE(1, "Skip invalid IMU data (timestamp=0)");
      return;
    }
    if (imu_msg->linear_acceleration_covariance[0] <= 0 || imu_msg->angular_velocity_covariance[0] <= 0) {
      ROS_WARN_THROTTLE(1, "Skip invalid IMU data (covariance=0)");
      return;
    }

    // 转换数据格式
    ImuData data;
    data.timestamp = timestamp;
    data.accel << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z;
    data.gyro << imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;

    // 维护滑动窗口（超出大小则弹出最早数据）
    imu_window_.push_back(data);
    if (imu_window_.size() > params_.window_size) {
      imu_window_.erase(imu_window_.begin());
    }

    // 计算统计值并发布清洗后数据
    Eigen::Vector3d accel_mean, accel_std, gyro_mean, gyro_std;
    if (calculateStats(imu_window_, accel_mean, accel_std, gyro_mean, gyro_std)) {
      // 过滤异常值（超过标准差阈值则用均值替代）
      Eigen::Vector3d clean_accel = accel_mean;
      Eigen::Vector3d clean_gyro = gyro_mean;
      for (int i = 0; i < 3; ++i) {
        if (std::fabs(data.accel[i] - accel_mean[i]) < params_.std_threshold * accel_std[i]) {
          clean_accel[i] = data.accel[i];
        }
        if (std::fabs(data.gyro[i] - gyro_mean[i]) < params_.std_threshold * gyro_std[i]) {
          clean_gyro[i] = data.gyro[i];
        }
      }

      sensor_msgs::Imu clean_msg;
      clean_msg.header = imu_msg->header; 
      
      
      clean_msg.linear_acceleration.x = clean_accel[0];
      clean_msg.linear_acceleration.y = clean_accel[1];
      clean_msg.linear_acceleration.z = clean_accel[2];
      if (params_.gyro_to_deg) {
        clean_msg.angular_velocity.x = clean_gyro[0] * 180 / M_PI;
        clean_msg.angular_velocity.y = clean_gyro[1] * 180 / M_PI;
        clean_msg.angular_velocity.z = clean_gyro[2] * 180 / M_PI;
      } else {
        clean_msg.angular_velocity.x = clean_gyro[0];
        clean_msg.angular_velocity.y = clean_gyro[1];
        clean_msg.angular_velocity.z = clean_gyro[2];
      }

      // 复用原始协方差
      clean_msg.linear_acceleration_covariance = imu_msg->linear_acceleration_covariance;
      clean_msg.angular_velocity_covariance = imu_msg->angular_velocity_covariance;
      clean_msg.orientation_covariance = imu_msg->orientation_covariance;

      // 发布清洗后数据
      imu_pub_.publish(clean_msg);
      ROS_DEBUG("Published cleaned IMU data (timestamp: %.6f)", timestamp);
    }
  }

public:
  ImuCleaner() : nh_("~") {
    // 加载初始参数
    nh_.param<int>("window_size", params_.window_size, 10);
    nh_.param<double>("std_threshold", params_.std_threshold, 0.1);
    nh_.param<bool>("gyro_to_deg", params_.gyro_to_deg, false);

    // 初始化动态参数服务器
    dynamic_reconfigure::Server<imu_clean::ImuCleanConfig>::CallbackType f;
    f = boost::bind(&ImuCleaner::reconfigCallback, this, _1, _2);
    dyn_srv_.setCallback(f);

    // 订阅原始IMU话题，设置队列大小
    int queue_size;
    nh_.param<int>("queue_size", queue_size, 100);
    imu_sub_ = nh_.subscribe("imu_raw", queue_size, &ImuCleaner::imuCallback, this);

    // 发布清洗后IMU话题
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_clean", queue_size);

    ROS_INFO("ImuCleaner initialized: window_size=%d, std_threshold=%.2f, gyro_to_deg=%d",
             params_.window_size, params_.std_threshold, params_.gyro_to_deg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_clean_node");
  ImuCleaner cleaner;
  ROS_INFO("IMU clean node started, waiting for IMU data...");
  ros::spin();
  return 0;
}