import numpy as np
from collections import deque
import threading

class IMUData:

    """IMU数据结构：支持6/9轴"""

    def __init__(self, is_9axis=False):
        self.timestamp = 0.0          # 时间戳（秒）
        self.accel = np.zeros(3)      # 加速度 (m/s²)
        self.gyro = np.zeros(3)       # 角速度 (rad/s)
        self.mag = np.zeros(3)        # 磁力计 (uT) - 9轴专用
        self.is_valid = True          # 原始数据是否有效
        self.is_filled = False        # 是否是前一帧填充的数据
        self.is_9axis = is_9axis      # 是否为9轴IMU

class IMUCleanConfig:

    """IMU清洗配置类"""

    def __init__(self):
        # 基础配置
        self.is_9axis = False             # 是否处理9轴数据
        self.sliding_window_size = 20     # 滑动窗口大小
        self.sigma_multiplier = 3.0       # 3σ倍数

        # 绝对阈值（兜底）
        self.accel_abs_thresh = 20.0      # 加速度绝对阈值 (m/s²)
        self.gyro_abs_thresh = 5.0        # 角速度绝对阈值 (rad/s)
        self.mag_abs_thresh = 500.0       # 磁力计绝对阈值 (uT)

class IMUDataCleaner:

    """IMU数据清洗类"""

    def __init__(self, config):

        self.config = config
        # 滑动窗口：存储最近N帧有效数据
        self.valid_data_window = deque(maxlen=config.sliding_window_size)
        # 前一帧有效数据（用于填充）
        self.last_valid_data = IMUData(config.is_9axis)
        # 线程锁（保证多线程安全）
        self.lock = threading.Lock()

    def reset(self):

        """重置清洗器"""

        with self.lock:
            self.valid_data_window.clear()
            self.last_valid_data = IMUData(self.config.is_9axis)

    def _calculate_stats(self, data_type):

        """计算滑动窗口内数据的均值和标准差"""

        with self.lock:
            if not self.valid_data_window:
                return np.zeros(3), np.zeros(3)
            
            # 提取窗口内对应数据
            data_list = []
            for data in self.valid_data_window:
                if data_type == "accel":
                    data_list.append(data.accel)
                elif data_type == "gyro":
                    data_list.append(data.gyro)
                elif data_type == "mag":
                    data_list.append(data.mag)
                else:
                    raise ValueError(f"无效数据类型：{data_type}")
            
            data_array = np.array(data_list)
            mean = np.mean(data_array, axis=0)
            std_dev = np.std(data_array, axis=0)
            return mean, std_dev

    def _is_axis_outlier(self, current_val, mean_val, std_dev_val, abs_thresh):

        """单轴野值判断：绝对阈值+3σ"""

        # 绝对阈值兜底
        if abs(current_val) > abs_thresh:
            return True
        # 3σ统计阈值
        stat_thresh = self.config.sigma_multiplier * std_dev_val
        if abs(current_val - mean_val) > stat_thresh:
            return True
        return False

    def _is_data_outlier(self, data, data_type):

        """检测单类数据（accel/gyro/mag）是否为野值"""

        mean, std_dev = self._calculate_stats(data_type)
        abs_thresh = {
            "accel": self.config.accel_abs_thresh,
            "gyro": self.config.gyro_abs_thresh,
            "mag": self.config.mag_abs_thresh
        }[data_type]

        # 窗口数据不足时，仅用绝对阈值
        if len(self.valid_data_window) < self.config.sliding_window_size // 2:
            return np.any(np.abs(getattr(data, data_type)) > abs_thresh)
        
        # 窗口充足：绝对阈值+3σ
        for i in range(3):
            current_val = getattr(data, data_type)[i]
            if self._is_axis_outlier(current_val, mean[i], std_dev[i], abs_thresh):
                return True
        return False

    def clean_imu_data(self, raw_data):

        """核心清洗方法：野值检测+前一帧填充"""
        
        with self.lock:
            cleaned_data = IMUData(raw_data.is_9axis)
            cleaned_data.timestamp = raw_data.timestamp
            cleaned_data.accel = raw_data.accel.copy()
            cleaned_data.gyro = raw_data.gyro.copy()
            if raw_data.is_9axis:
                cleaned_data.mag = raw_data.mag.copy()
            cleaned_data.is_filled = False

            # 初始化：无前一帧有效数据时，保存首帧有效数据
            if self.last_valid_data.timestamp == 0.0:
                if not self._is_data_outlier(raw_data, "accel") and not self._is_data_outlier(raw_data, "gyro"):
                    self.last_valid_data = raw_data
                    self.valid_data_window.append(raw_data)
                    cleaned_data.is_valid = True
                    return cleaned_data
                else:
                    cleaned_data.is_valid = False
                    return cleaned_data

            # 野值检测
            is_accel_outlier = self._is_data_outlier(raw_data, "accel")
            is_gyro_outlier = self._is_data_outlier(raw_data, "gyro")
            is_mag_outlier = False
            if raw_data.is_9axis:
                is_mag_outlier = self._is_data_outlier(raw_data, "mag")

            # 任一轴异常则填充
            is_total_outlier = is_accel_outlier or is_gyro_outlier or (raw_data.is_9axis and is_mag_outlier)
            if is_total_outlier:
                cleaned_data.is_valid = False
                cleaned_data.accel = self.last_valid_data.accel.copy()
                cleaned_data.gyro = self.last_valid_data.gyro.copy()
                if raw_data.is_9axis:
                    cleaned_data.mag = self.last_valid_data.mag.copy()
                cleaned_data.is_filled = True
                return cleaned_data

            # 非野值：更新窗口和前一帧
            cleaned_data.is_valid = True
            self.last_valid_data = cleaned_data
            self.valid_data_window.append(cleaned_data)
            return cleaned_data