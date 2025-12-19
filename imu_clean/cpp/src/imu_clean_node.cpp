#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "imu_clean\cpp\include\imu_clean\imu_data_cleaner.h"
#include <memory>

// 封装ROS节点逻辑：避免全局指针，支持6/9轴
class IMUCleanNode {
public:
    IMUCleanNode(ros::NodeHandle& nh) : nh_(nh) {
        // 1. 从ROS参数服务器读取配置（优先），无则用手写默认值
        loadConfig();

        // 2. 初始化清洗器
        cleaner_ = std::make_unique<IMUDataCleaner>(config_);

        // 3. 订阅话题（6轴：仅IMU；9轴：IMU+磁力计）
        sub_imu_ = nh_.subscribe("/imu/data_raw", 1000, &IMUCleanNode::imuCallback, this);
        if (config_.is_9axis) {
            sub_mag_ = nh_.subscribe("/imu/mag", 1000, &IMUCleanNode::magCallback, this);
        }

        // 4. 发布清洗后的数据
        pub_cleaned_imu_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_cleaned", 1000);
        if (config_.is_9axis) {
            pub_cleaned_mag_ = nh_.advertise<sensor_msgs::MagneticField>("/imu/mag_cleaned", 1000);
        }

        ROS_INFO("IMU清洗节点启动成功！");
        ROS_INFO("配置：%d轴 | 滑动窗口大小：%d | 3σ倍数：%.1f",
                 config_.is_9axis ? 9 : 6,
                 config_.sliding_window_size,
                 config_.sigma_multiplier);
    }

    // IMU回调（6/9轴通用）
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 1. 构造原始IMU数据（6/9轴）
        IMUData raw_data(config_.is_9axis);
        raw_data.timestamp = msg->header.stamp.toSec();
        raw_data.accel << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        raw_data.gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

        // 9轴：补充磁力计数据（需先接收mag回调）
        if (config_.is_9axis) {
            std::lock_guard<std::mutex> lock(mag_mutex_);
            if (last_mag_timestamp_ > 0.0) {
                raw_data.mag = last_mag_data_;
            } else {
                ROS_WARN_THROTTLE(1.0, "9轴模式下无磁力计数据！");
            }
        }

        // 2. 调用清洗方法
        IMUData cleaned_data = cleaner_->cleanIMUData(raw_data);

        // 3. 发布清洗后的数据
        publishCleanedIMU(cleaned_data, msg->header.frame_id);

        // 4. 日志输出（节流1秒）
        logCleanedData(cleaned_data);
    }

    // 磁力计回调（9轴专用）
    void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mag_mutex_);
        last_mag_data_ << msg->magnetic_field.x, msg->magnetic_field.y, msg->magnetic_field.z;
        last_mag_timestamp_ = msg->header.stamp.toSec();
    }

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_mag_;
    ros::Publisher pub_cleaned_imu_;
    ros::Publisher pub_cleaned_mag_;

    // 清洗器（智能指针：自动释放）
    std::unique_ptr<IMUDataCleaner> cleaner_;

    // 配置
    IMUCleanConfig config_;

    // 9轴磁力计缓存
    std::mutex mag_mutex_;
    Eigen::Vector3d last_mag_data_;
    double last_mag_timestamp_ = 0.0;

    // 加载配置：优先ROS参数，无则手写默认值
    void loadConfig() {
        // 【用户可直接修改此处手写默认值】
        config_.is_9axis = false;          // 默认6轴，改为true则9轴
        config_.sliding_window_size = 20;  // 滑动窗口大小
        config_.sigma_multiplier = 3.0;    // 3σ倍数

        // 绝对阈值（兜底）
        config_.accel_abs_thresh = 15.0;   // 加速度绝对阈值
        config_.gyro_abs_thresh = 4.0;     // 角速度绝对阈值
        config_.mag_abs_thresh = 500.0;    // 磁力计绝对阈值

        // 从ROS参数服务器读取
        nh_.param("imu_clean/is_9axis", config_.is_9axis, config_.is_9axis);
        nh_.param("imu_clean/sliding_window_size", config_.sliding_window_size, config_.sliding_window_size);
        nh_.param("imu_clean/sigma_multiplier", config_.sigma_multiplier, config_.sigma_multiplier);
        nh_.param("imu_clean/accel_abs_thresh", config_.accel_abs_thresh, config_.accel_abs_thresh);
        nh_.param("imu_clean/gyro_abs_thresh", config_.gyro_abs_thresh, config_.gyro_abs_thresh);
        nh_.param("imu_clean/mag_abs_thresh", config_.mag_abs_thresh, config_.mag_abs_thresh);
    }

    // 发布清洗后的IMU数据
    void publishCleanedIMU(const IMUData& cleaned_data, const std::string& frame_id) {
        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time(cleaned_data.timestamp);
        msg.header.frame_id = frame_id;

        // 填充清洗后的数据
        msg.linear_acceleration.x = cleaned_data.accel.x();
        msg.linear_acceleration.y = cleaned_data.accel.y();
        msg.linear_acceleration.z = cleaned_data.accel.z();
        msg.angular_velocity.x = cleaned_data.gyro.x();
        msg.angular_velocity.y = cleaned_data.gyro.y();
        msg.angular_velocity.z = cleaned_data.gyro.z();

        // 发布IMU
        pub_cleaned_imu_.publish(msg);

        // 9轴：发布磁力计
        if (config_.is_9axis) {
            sensor_msgs::MagneticField mag_msg;
            mag_msg.header.stamp = ros::Time(cleaned_data.timestamp);
            mag_msg.header.frame_id = frame_id;
            mag_msg.magnetic_field.x = cleaned_data.mag.x();
            mag_msg.magnetic_field.y = cleaned_data.mag.y();
            mag_msg.magnetic_field.z = cleaned_data.mag.z();
            pub_cleaned_mag_.publish(mag_msg);
        }
    }

    // 日志输出
    void logCleanedData(const IMUData& cleaned_data) {
        if (cleaned_data.is_filled) {
            ROS_WARN_THROTTLE(1.0, "检测到IMU野值，已用前一帧填充！");
        } else {
            ROS_INFO_THROTTLE(1.0, "有效IMU数据 | 加速度x=%.2f | 角速度x=%.2f | %d轴",
                              cleaned_data.accel.x(),
                              cleaned_data.gyro.x(),
                              config_.is_9axis ? 9 : 6);
        }
    }
};

// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_clean_node");
    ros::NodeHandle nh("~"); // 私有命名空间，方便参数配置

    // 初始化节点
    IMUCleanNode node(nh);

    ros::spin();
    return 0;
}