#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Imu, MagneticField
from imu_clean.imu_data_cleaner import IMUData, IMUCleanConfig, IMUDataCleaner
import threading

class IMUCleanNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('imu_clean_node_py', anonymous=True)
        
        # 加载配置（优先ROS参数，无则用默认值）
        self.config = self._load_config()
        
        # 初始化清洗器
        self.cleaner = IMUDataCleaner(self.config)
        
        # 订阅话题
        self.imu_sub = rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback, queue_size=1000)
        self.mag_sub = None
        if self.config.is_9axis:
            self.mag_sub = rospy.Subscriber('/imu/mag', MagneticField, self.mag_callback, queue_size=1000)
            # 磁力计数据缓存（加锁）
            self.mag_data = np.zeros(3)
            self.mag_timestamp = 0.0
            self.mag_lock = threading.Lock()
        
        # 发布清洗后的数据
        self.imu_pub = rospy.Publisher('/imu/data_cleaned', Imu, queue_size=1000)
        self.mag_pub = None
        if self.config.is_9axis:
            self.mag_pub = rospy.Publisher('/imu/mag_cleaned', MagneticField, queue_size=1000)
        
        # 日志
        rospy.loginfo(f"Python版IMU清洗节点启动成功！")
        rospy.loginfo(f"配置：{9 if self.config.is_9axis else 6}轴 | 滑动窗口：{self.config.sliding_window_size} | 3σ倍数：{self.config.sigma_multiplier}")

    def _load_config(self):

        """加载配置"""

        config = IMUCleanConfig()
        
        # 从ROS参数服务器读取（覆盖默认值）
        config.is_9axis = rospy.get_param("~is_9axis", config.is_9axis)
        config.sliding_window_size = rospy.get_param("~sliding_window_size", config.sliding_window_size)
        config.sigma_multiplier = rospy.get_param("~sigma_multiplier", config.sigma_multiplier)
        config.accel_abs_thresh = rospy.get_param("~accel_abs_thresh", config.accel_abs_thresh)
        config.gyro_abs_thresh = rospy.get_param("~gyro_abs_thresh", config.gyro_abs_thresh)
        config.mag_abs_thresh = rospy.get_param("~mag_abs_thresh", config.mag_abs_thresh)
        
        return config

    def imu_callback(self, msg):
        """IMU回调函数"""
        # 构造原始IMU数据
        raw_data = IMUData(self.config.is_9axis)
        raw_data.timestamp = msg.header.stamp.to_sec()
        raw_data.accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        raw_data.gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        
        # 9轴：补充磁力计数据
        if self.config.is_9axis:
            with self.mag_lock:
                if self.mag_timestamp > 0:
                    raw_data.mag = self.mag_data.copy()
                else:
                    rospy.logwarn_throttle(1.0, "9轴模式下无磁力计数据！")
        
        # 清洗数据
        cleaned_data = self.cleaner.clean_imu_data(raw_data)
        
        # 发布清洗后的IMU数据
        self._publish_cleaned_imu(cleaned_data, msg.header.frame_id)
        
        # 9轴：发布清洗后的磁力计
        if self.config.is_9axis and self.mag_pub:
            self._publish_cleaned_mag(cleaned_data, msg.header.frame_id)
        
        # 日志输出
        self._log_cleaned_data(cleaned_data)

    def mag_callback(self, msg):

        """磁力计回调函数（9轴）"""

        with self.mag_lock:
            self.mag_data = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
            self.mag_timestamp = msg.header.stamp.to_sec()

    def _publish_cleaned_imu(self, cleaned_data, frame_id):

        """发布清洗后的IMU消息"""

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.from_sec(cleaned_data.timestamp)
        imu_msg.header.frame_id = frame_id
        
        # 填充清洗后的数据
        imu_msg.linear_acceleration.x = cleaned_data.accel[0]
        imu_msg.linear_acceleration.y = cleaned_data.accel[1]
        imu_msg.linear_acceleration.z = cleaned_data.accel[2]
        imu_msg.angular_velocity.x = cleaned_data.gyro[0]
        imu_msg.angular_velocity.y = cleaned_data.gyro[1]
        imu_msg.angular_velocity.z = cleaned_data.gyro[2]
        
        self.imu_pub.publish(imu_msg)

    def _publish_cleaned_mag(self, cleaned_data, frame_id):

        """发布清洗后的磁力计消息"""

        mag_msg = MagneticField()
        mag_msg.header.stamp = rospy.Time.from_sec(cleaned_data.timestamp)
        mag_msg.header.frame_id = frame_id
        
        mag_msg.magnetic_field.x = cleaned_data.mag[0]
        mag_msg.magnetic_field.y = cleaned_data.mag[1]
        mag_msg.magnetic_field.z = cleaned_data.mag[2]
        
        self.mag_pub.publish(mag_msg)

    def _log_cleaned_data(self, cleaned_data):

        """日志输出（节流）"""

        if cleaned_data.is_filled:
            rospy.logwarn_throttle(1.0, "检测到IMU野值，已用前一帧填充！")
        else:
            rospy.loginfo_throttle(1.0, 
                f"有效IMU数据 | 加速度x={cleaned_data.accel[0]:.2f} | 角速度x={cleaned_data.gyro[0]:.2f} | {9 if self.config.is_9axis else 6}轴"
            )

    def run(self):

        """节点运行"""
        
        rospy.spin()

if __name__ == '__main__':
    try:
        node = IMUCleanNode()
        node.run()
    except rospy.ROSInterruptException:
        pass