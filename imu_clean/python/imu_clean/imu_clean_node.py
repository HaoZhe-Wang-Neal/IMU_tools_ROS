from __future__ import division, print_function, unicode_literals

import rospy
import threading
import math
import numpy as np
from sensor_msgs.msg import Imu
from collections import deque

class ImuCleaner:
    def __init__(self):
        # 私有命名空间
        self.nh = rospy.get_param("~")

        self.window_size = rospy.get_param("~window_size", 10)
        self.std_threshold = rospy.get_param("~std_threshold", 0.1)
        self.gyro_to_deg = rospy.get_param("~gyro_to_deg", False)
        self.queue_size = rospy.get_param("~queue_size", 100)
        

        self.accel_sum = np.zeros(3)  # 加速度累加和
        self.gyro_sum = np.zeros(3)   # 陀螺仪累加和
        self.accel_sq_sum = np.zeros(3) # 加速度平方和
        self.gyro_sq_sum = np.zeros(3)  # 陀螺仪平方和
        self.imu_window = deque(maxlen=self.window_size) # 固定长度队列
        

        self.lock = threading.Lock()
        
        # 订阅/发布话题（支持重映射）
        self.imu_sub = rospy.Subscriber("imu_raw", Imu, self.imu_callback, queue_size=self.queue_size)
        self.imu_pub = rospy.Publisher("imu_clean", Imu, queue_size=self.queue_size)
        
        rospy.loginfo("ImuCleaner initialized:")
        rospy.loginfo("  window_size: %d", self.window_size)
        rospy.loginfo("  std_threshold: %.2f", self.std_threshold)
        rospy.loginfo("  gyro_to_deg: %s", self.gyro_to_deg)
        rospy.loginfo("  subscribe: %s, publish: %s", self.imu_sub.resolved_name, self.imu_pub.resolved_name)

    def calculate_stats(self):
        """修复：累加器计算均值+标准差，提升效率"""

        # 边界条件检查

        if len(self.imu_window) < self.window_size:
            rospy.logwarn_throttle(1, "Window not full (current: %d, required: %d)", len(self.imu_window), self.window_size)
            return None, None, None, None
        
        # 计算均值
        accel_mean = self.accel_sum / len(self.imu_window)
        gyro_mean = self.gyro_sum / len(self.imu_window)
        
        # 计算标准差（E[X²] - (E[X])² 的平方根）
        accel_std = np.sqrt((self.accel_sq_sum / len(self.imu_window)) - (accel_mean **2))
        gyro_std = np.sqrt((self.gyro_sq_sum / len(self.imu_window)) - (gyro_mean** 2))
        
        return accel_mean, accel_std, gyro_mean, gyro_std

    def imu_callback(self, imu_msg):
        with self.lock: 
            # 过滤无效数据
            timestamp = imu_msg.header.stamp.to_sec()
            if timestamp <= 0:
                rospy.logwarn_throttle(1, "Skip invalid IMU data (timestamp=0)")
                return
            if imu_msg.linear_acceleration_covariance[0] <= 0 or imu_msg.angular_velocity_covariance[0] <= 0:
                rospy.logwarn_throttle(1, "Skip invalid IMU data (covariance=0)")
                return
            

            if len(self.imu_window) > 0:
                last_ts = self.imu_window[-1]["timestamp"]
                if timestamp - last_ts > 0.1: # 超过0.1s判定为丢包
                    rospy.logwarn("IMU data lost (time gap: %.2fs), clear window", timestamp - last_ts)
                    self.imu_window.clear()
                    self.accel_sum = np.zeros(3)
                    self.gyro_sum = np.zeros(3)
                    self.accel_sq_sum = np.zeros(3)
                    self.gyro_sq_sum = np.zeros(3)
                    return
            
            # 提取数据
            accel = np.array([imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z])
            gyro = np.array([imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z])
            
            # 维护累加器（新增数据）
            self.accel_sum += accel
            self.gyro_sum += gyro
            self.accel_sq_sum += accel **2
            self.gyro_sq_sum += gyro** 2
            
            # 加入窗口
            self.imu_window.append({
                "timestamp": timestamp,
                "accel": accel,
                "gyro": gyro
            })
            
            # 维护累加器（移除超出窗口的数据）
            if len(self.imu_window) > self.window_size:
                popped = self.imu_window.popleft()
                self.accel_sum -= popped["accel"]
                self.gyro_sum -= popped["gyro"]
                self.accel_sq_sum -= popped["accel"] **2
                self.gyro_sq_sum -= popped["gyro"]** 2
            
            # 计算统计值并发布清洗后数据
            accel_mean, accel_std, gyro_mean, gyro_std = self.calculate_stats()
            if accel_mean is None:
                return
            
            # 过滤异常值
            clean_accel = np.where(np.abs(accel - accel_mean) < self.std_threshold * accel_std, accel, accel_mean)
            clean_gyro = np.where(np.abs(gyro - gyro_mean) < self.std_threshold * gyro_std, gyro, gyro_mean)
            
            # 构建清洗后消息
            clean_msg = Imu()
            clean_msg.header = imu_msg.header # 复用原始header
            clean_msg.linear_acceleration.x = clean_accel[0]
            clean_msg.linear_acceleration.y = clean_accel[1]
            clean_msg.linear_acceleration.z = clean_accel[2]
            
 
 
            if self.gyro_to_deg:
                clean_msg.angular_velocity.x = clean_gyro[0] * 180 / math.pi
                clean_msg.angular_velocity.y = clean_gyro[1] * 180 / math.pi
                clean_msg.angular_velocity.z = clean_gyro[2] * 180 / math.pi
            else:
                clean_msg.angular_velocity.x = clean_gyro[0]
                clean_msg.angular_velocity.y = clean_gyro[1]
                clean_msg.angular_velocity.z = clean_gyro[2]
            
            # 复用原始协方差
            clean_msg.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
            clean_msg.angular_velocity_covariance = imu_msg.angular_velocity_covariance
            clean_msg.orientation_covariance = imu_msg.orientation_covariance
            
            # 发布数据
            self.imu_pub.publish(clean_msg)

def main():
    rospy.init_node("imu_clean_node", anonymous=True)
    try:
        cleaner = ImuCleaner()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("IMU clean node interrupted")
    except Exception as e:
        rospy.logerr("IMU clean node error: %s", str(e))

if __name__ == "__main__":
    main()