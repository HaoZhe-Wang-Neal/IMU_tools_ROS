#include "imu_clean\cpp\include\imu_clean\imu_data_cleaner.h"
#include <Eigen/Dense>
#include <stdexcept>
#include <numeric>

// 构造函数：初始化配置和窗口
IMUDataCleaner::IMUDataCleaner(const IMUCleanConfig& config) : config_(config) {
    // 初始化前一帧数据（轴数与配置一致）
    last_valid_data_ = IMUData(config_.is_9axis);
}

// 重置清洗器
void IMUDataCleaner::reset() {
    valid_data_window_.clear();
    last_valid_data_ = IMUData(config_.is_9axis);
}

// 计算滑动窗口内数据的均值和标准差
void IMUDataCleaner::calculateStats(const std::deque<IMUData>& window, 
                                    const std::string& data_type,
                                    Eigen::Vector3d& mean, 
                                    Eigen::Vector3d& std_dev) {
    if (window.empty()) {
        mean.setZero();
        std_dev.setZero();
        return;
    }

    // 1. 计算均值
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto& data : window) {
        if (data_type == "accel") sum += data.accel;
        else if (data_type == "gyro") sum += data.gyro;
        else if (data_type == "mag") sum += data.mag;
        else throw std::invalid_argument("无效的数据类型：" + data_type);
    }
    mean = sum / static_cast<double>(window.size());

    // 2. 计算标准差
    Eigen::Vector3d sq_sum = Eigen::Vector3d::Zero();
    for (const auto& data : window) {
        Eigen::Vector3d diff;
        if (data_type == "accel") diff = data.accel - mean;
        else if (data_type == "gyro") diff = data.gyro - mean;
        else if (data_type == "mag") diff = data.mag - mean;
        sq_sum += diff.cwiseProduct(diff); // 元素-wise平方和
    }
    std_dev = (sq_sum / static_cast<double>(window.size())).cwiseSqrt();
}

// 单轴野值判断：绝对阈值（兜底） + 统计阈值（3σ）
bool IMUDataCleaner::isAxisOutlier(double current_val, double mean_val, double std_dev_val, 
                                   double abs_thresh) {
    // 1. 绝对阈值判断（优先：窗口无数据/极端值）
    if (std::fabs(current_val) > abs_thresh) {
        return true;
    }

    // 2. 统计阈值判断（3σ原则：超出均值±sigma倍标准差）
    double stat_thresh = config_.sigma_multiplier * std_dev_val;
    if (std::fabs(current_val - mean_val) > stat_thresh) {
        return true;
    }

    return false;
}

// 检测单类数据（accel/gyro/mag）是否为野值
bool IMUDataCleaner::isDataOutlier(const IMUData& data, const std::string& data_type) {
    Eigen::Vector3d mean, std_dev;
    calculateStats(valid_data_window_, data_type, mean, std_dev);

    double abs_thresh = 0.0;
    if (data_type == "accel") abs_thresh = config_.accel_abs_thresh;
    else if (data_type == "gyro") abs_thresh = config_.gyro_abs_thresh;
    else if (data_type == "mag") abs_thresh = config_.mag_abs_thresh;

    // 检查三轴是否有任一轴为野值
    bool is_outlier = false;
    for (int i = 0; i < 3; ++i) {
        double current_val = 0.0;
        if (data_type == "accel") current_val = data.accel(i);
        else if (data_type == "gyro") current_val = data.gyro(i);
        else if (data_type == "mag") current_val = data.mag(i);

        // 窗口数据不足时，仅用绝对阈值
        if (valid_data_window_.size() < config_.sliding_window_size / 2) {
            if (std::fabs(current_val) > abs_thresh) {
                is_outlier = true;
                break;
            }
        } else {
            // 窗口数据充足：绝对阈值+统计阈值
            if (isAxisOutlier(current_val, mean(i), std_dev(i), abs_thresh)) {
                is_outlier = true;
                break;
            }
        }
    }
    return is_outlier;
}

// 核心清洗方法：野值检测 + 前一帧填充
IMUData IMUDataCleaner::cleanIMUData(const IMUData& raw_data) {
    IMUData cleaned_data = raw_data;
    cleaned_data.is_filled = false;

    // 1. 初始化检查：若无前一帧有效数据，且当前数据有效 → 保存为前一帧
    if (last_valid_data_.timestamp == 0.0 && !isDataOutlier(raw_data, "accel") && !isDataOutlier(raw_data, "gyro")) {
        last_valid_data_ = raw_data;
        valid_data_window_.push_back(raw_data);
        return cleaned_data;
    }

    // 2. 野值检测（6轴：accel+gyro；9轴：+mag）
    bool is_accel_outlier = isDataOutlier(raw_data, "accel");
    bool is_gyro_outlier = isDataOutlier(raw_data, "gyro");
    bool is_mag_outlier = false;
    if (config_.is_9axis) {
        is_mag_outlier = isDataOutlier(raw_data, "mag");
    }

    // 3. 判断是否为野值（任一轴异常则标记为野值）
    bool is_total_outlier = is_accel_outlier || is_gyro_outlier || (config_.is_9axis && is_mag_outlier);
    if (is_total_outlier) {
        cleaned_data.is_valid = false;
        // 用前一帧有效数据填充
        cleaned_data.accel = last_valid_data_.accel;
        cleaned_data.gyro = last_valid_data_.gyro;
        if (config_.is_9axis) {
            cleaned_data.mag = last_valid_data_.mag;
        }
        cleaned_data.is_filled = true;
        return cleaned_data;
    }

    // 4. 非野值：更新滑动窗口和前一帧数据
    cleaned_data.is_valid = true;
    last_valid_data_ = cleaned_data;
    valid_data_window_.push_back(cleaned_data);

    // 5. 维护滑动窗口大小（超过配置则弹出最旧数据）
    if (valid_data_window_.size() > config_.sliding_window_size) {
        valid_data_window_.pop_front();
    }

    return cleaned_data;
}