#ifndef IMU_DATA_CLEANER_H
#define IMU_DATA_CLEANER_H

#include <Eigen/Core>
#include <deque>
#include <vector>
#include <cmath>

// 支持6轴(accel+gyro) / 9轴(+mag)
struct IMUData {
    double timestamp;          // 时间戳（秒）
    Eigen::Vector3d accel;     // 加速度 (m/s²)
    Eigen::Vector3d gyro;      // 角速度 (rad/s)
    Eigen::Vector3d mag;       // 磁力计 (uT) - 9轴专用
    bool is_valid = true;      // 原始数据是否有效
    bool is_filled = false;    // 是否是前一帧填充的数据
    bool is_9axis = false;     // 是否为9轴IMU数据（false=6轴）

    // 构造函数：默认6轴
    IMUData() : is_9axis(false) {}
    // 构造函数：指定轴数
    IMUData(bool is_9axis) : is_9axis(is_9axis) {}
};

// IMU清洗配置结构体：支持动态配置参数
struct IMUCleanConfig {

    // 基础配置
    bool is_9axis = false;             // 是否处理9轴数据（false=6轴）
    int sliding_window_size = 20;      // 滑动窗口大小
    double sigma_multiplier = 3.0;     // 3σ原则的倍数（统计学异常值判断）

    // 绝对阈值（窗口初始化/极端野值）
    double accel_abs_thresh = 20.0;    // 加速度绝对阈值 (m/s²)
    double gyro_abs_thresh = 5.0;      // 角速度绝对阈值 (rad/s)
    double mag_abs_thresh = 500.0;     // 磁力计绝对阈值 (uT) - 9轴专用

    // 统计阈值允许的最大偏差（均值±sigma_multiplier*标准差）
    double accel_stat_thresh_ratio = 0.5; // 加速度统计偏差比例（可选：替代固定σ）
    double gyro_stat_thresh_ratio = 0.5;  // 角速度统计偏差比例
    double mag_stat_thresh_ratio = 0.5;   // 磁力计统计偏差比例 - 9轴专用
};

// IMU数据清洗类：支持6/9轴、滑动窗口统计阈值、野值填充
class IMUDataCleaner {
public:
    // 构造函数：传入动态配置
    IMUDataCleaner(const IMUCleanConfig& config);

    // 核心方法：清洗IMU数据（野值检测+填充）
    IMUData cleanIMUData(const IMUData& raw_data);

    // 重置清洗器（清空窗口和前一帧数据）
    void reset();

private:
    // 配置参数
    IMUCleanConfig config_;

    // 滑动窗口：存储最近N帧有效数据（用于统计均值/标准差）
    std::deque<IMUData> valid_data_window_;

    // 前一帧有效数据（用于野值填充）
    IMUData last_valid_data_;

    // 内部方法：计算滑动窗口内某类数据的均值和标准差
    void calculateStats(const std::deque<IMUData>& window, 
                        const std::string& data_type,  // "accel"/"gyro"/"mag"
                        Eigen::Vector3d& mean, 
                        Eigen::Vector3d& std_dev);

    // 内部方法：单轴野值判断（绝对阈值+统计阈值）
    bool isAxisOutlier(double current_val, double mean_val, double std_dev_val, 
                       double abs_thresh);

    // 内部方法：检测单类数据（accel/gyro/mag）是否为野值
    bool isDataOutlier(const IMUData& data, const std::string& data_type);
};

#endif // IMU_DATA_CLEANER_H