#!/usr/bin/env python3
import rospy
import os
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from imu_motion_monitor.msg import ImuAnomaly
from imu_motion_monitor import ImuMotionMonitorCore

class ImuMotionMonitorNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('imu_motion_monitor_node_py', anonymous=True)
        
        # 获取配置文件路径
        config_path = rospy.get_param("~config_path", 
            os.path.join(rospy.get_package_dir("imu_motion_monitor"), "config", "imu_motion_monitor.yaml"))
        
        # 初始化核心检测类
        try:
            self.core = ImuMotionMonitorCore(config_path)
        except Exception as e:
            rospy.logerr(f"Failed to initialize core: {e}")
            rospy.signal_shutdown("Init failed")
            return
        
        # 订阅IMU话题
        self.imu_sub = rospy.Subscriber(
            self.core.config.imu_topic, 
            Imu, 
            self.imu_callback, 
            queue_size=100
        )
        
        # 创建发布器
        if self.core.config.anomaly_msg_type == "bool":
            self.anomaly_pub = rospy.Publisher(
                self.core.config.anomaly_topic,
                Bool,
                queue_size=10
            )
        elif self.core.config.anomaly_msg_type == "custom":
            self.anomaly_pub = rospy.Publisher(
                self.core.config.anomaly_topic,
                ImuAnomaly,
                queue_size=10
            )
        
        rospy.loginfo(f"IMU Motion Monitor (Python) started for scene: {self.core.config.active_scene}")
        rospy.loginfo(f"Subscribed to: {self.core.config.imu_topic}")
        rospy.loginfo(f"Publishing to: {self.core.config.anomaly_topic} (type: {self.core.config.anomaly_msg_type})")

    def imu_callback(self, msg):
        """IMU数据回调"""
        # 处理数据并检测异常
        is_anomaly = self.core.process_imu_data(msg)
        
        # 仅异常时发布
        if is_anomaly:
            self.publish_anomaly(is_anomaly)
            rospy.loginfo(f"[{self.core.config.active_scene}] Detected anomaly: {self.core.config.anomaly_type}")

    def publish_anomaly(self, is_anomaly):
        """发布异常消息"""
        if self.core.config.anomaly_msg_type == "bool":
            msg = Bool()
            msg.data = is_anomaly
            self.anomaly_pub.publish(msg)
        elif self.core.config.anomaly_msg_type == "custom":
            msg = ImuAnomaly()
            msg.stamp = rospy.Time.now()
            msg.is_anomaly = is_anomaly
            msg.anomaly_type = self.core.config.anomaly_type
            self.anomaly_pub.publish(msg)

    def run(self):
        """节点主循环"""
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ImuMotionMonitorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass