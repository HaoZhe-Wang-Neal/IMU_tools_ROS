#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Header.h>
#include <fstream>
#include <string>
#include <vector>
#include <memory> 

// 全局参数结构体
struct Params {
  std::string bag_path;
  std::string csv_path;
  std::string imu_topic;
  std::string mag_topic;
  bool export_mag;
};

Params loadParams(ros::NodeHandle& nh) {
  Params params;
  nh.param<std::string>("bag_path", params.bag_path, "$(find bag2csv)/data/imu.bag");
  nh.param<std::string>("csv_path", params.csv_path, "$(find bag2csv)/output/imu_data.csv");
  nh.param<std::string>("imu_topic", params.imu_topic, "/imu/data_raw");
  nh.param<std::string>("mag_topic", params.mag_topic, "/imu/mag");
  nh.param<bool>("export_mag", params.export_mag, true);
  return params;
}

int main(int argc, char** argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "bag2csv_node");
  ros::NodeHandle nh("~"); // 私有命名空间
  Params params = loadParams(nh);

  rosbag::Bag bag;
  try {
    bag.open(params.bag_path, rosbag::bagmode::Read);
  } catch (const std::exception& e) {
    ROS_ERROR("Failed to open bag file: %s (error: %s)", params.bag_path.c_str(), e.what());
    return -1;
  }
  if (!bag.isOpen()) {
    ROS_ERROR("Bag file is not open: %s", params.bag_path.c_str());
    return -1;
  }

  std::vector<std::string> topics;
  topics.push_back(params.imu_topic);
  if (params.export_mag) {
    topics.push_back(params.mag_topic);
  }
  std::shared_ptr<rosbag::View> view = std::make_shared<rosbag::View>(bag, rosbag::TopicQuery(topics));

  std::ofstream csv_file(params.csv_path);
  if (!csv_file.is_open()) {
    ROS_ERROR("Failed to create CSV file (check permission): %s", params.csv_path.c_str());
    bag.close();
    return -1;
  }

  // 写入CSV表头
  csv_file << "timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z";
  if (params.export_mag) {
    csv_file << ",mag_x,mag_y,mag_z";
  }
  csv_file << std::endl;

  // 存储磁力计数据（按时间戳匹配）
  std::map<double, sensor_msgs::MagneticField::ConstPtr> mag_map;
  if (params.export_mag) {
    // 先读取所有磁力计数据到map
    for (const rosbag::MessageInstance& msg : *view) {
      if (msg.getTopic() == params.mag_topic) {
        sensor_msgs::MagneticField::ConstPtr mag_msg = msg.instantiate<sensor_msgs::MagneticField>();
        if (mag_msg) {
          double timestamp = mag_msg->header.stamp.toSec();
          mag_map[timestamp] = mag_msg;
        }
      }
    }
  }

  // 读取IMU数据并写入CSV
  int valid_count = 0;
  int invalid_count = 0;
  for (const rosbag::MessageInstance& msg : *view) {
    if (msg.getTopic() == params.imu_topic) {
      sensor_msgs::Imu::ConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
      if (!imu_msg) {
        invalid_count++;
        continue;
      }

      double timestamp = imu_msg->header.stamp.toSec();
      if (timestamp <= 0) {
        ROS_WARN("Skip invalid IMU data (timestamp=0)");
        invalid_count++;
        continue;
      }
      if (imu_msg->linear_acceleration_covariance[0] <= 0 || imu_msg->angular_velocity_covariance[0] <= 0) {
        ROS_WARN("Skip invalid IMU data (covariance=0, timestamp: %.6f)", timestamp);
        invalid_count++;
        continue;
      }

      // 写入IMU数据
      csv_file << timestamp << ","
               << imu_msg->linear_acceleration.x << ","
               << imu_msg->linear_acceleration.y << ","
               << imu_msg->linear_acceleration.z << ","
               << imu_msg->angular_velocity.x << ","
               << imu_msg->angular_velocity.y << ","
               << imu_msg->angular_velocity.z;

      // 匹配并写入磁力计数据
      if (params.export_mag && !mag_map.empty()) {
        // 找最接近的时间戳（误差±0.01s）
        double mag_timestamp = -1.0;
        sensor_msgs::MagneticField::ConstPtr mag_msg;
        for (const auto& pair : mag_map) {
          if (fabs(pair.first - timestamp) < 0.01) {
            mag_timestamp = pair.first;
            mag_msg = pair.second;
            break;
          }
        }
        if (mag_msg) {
          csv_file << "," << mag_msg->magnetic_field.x << ","
                   << mag_msg->magnetic_field.y << ","
                   << mag_msg->magnetic_field.z;
        } else {
          csv_file << ",0,0,0"; // 无匹配数据填0
          ROS_WARN("No mag data matched for IMU timestamp: %.6f", timestamp);
        }
      }
      csv_file << std::endl;
      valid_count++;
    }
  }

  // 清理资源
  csv_file.close();
  bag.close();
  view.reset(); 

  // 日志输出统计
  ROS_INFO("Bag2CSV finished!");
  ROS_INFO("Valid IMU data written: %d", valid_count);
  ROS_INFO("Invalid IMU data skipped: %d", invalid_count);
  ROS_INFO("CSV file saved to: %s", params.csv_path.c_str());

  return 0;
}