#!/usr/bin/env python3
import yaml
import math
from collections import deque

class MonitorConfig:
    """配置结构体"""
    def __init__(self):
        # 全局配置
        self.imu_topic = "/imu/data"
        self.window_size = 5
        self.anomaly_msg_type = "bool"
        
        # 场景配置
        self.active_scene = ""
        self.anomaly_topic = ""
        self.anomaly_type = ""
        self.detect_thresholds = {}

class ImuMotionMonitorCore:
    """核心异常检测逻辑"""
    def __init__(self, config_path):
        # 加载配置
        self.config = self.load_config(config_path)
        
        # 初始化滑动窗口
        self.imu_window = deque(maxlen=self.config.window_size)

    def load_config(self, config_path):
        """加载配置文件（全局+场景合并）"""
        config = MonitorConfig()
        
        # 读取YAML
        try:
            with open(config_path, 'r') as f:
                yaml_config = yaml.safe_load(f)
        except FileNotFoundError:
            raise Exception(f"Config file not found: {config_path}")
        
        # 加载全局配置
        global_config = yaml_config.get("global", {})
        config.imu_topic = global_config.get("imu_topic", "/imu/data")
        config.window_size = global_config.get("window_size", 5)
        config.anomaly_msg_type = global_config.get("anomaly_msg_type", "bool")
        
        # 加载激活场景
        config.active_scene = yaml_config.get("active_scene", "mobile_robot")
        scene_config = yaml_config.get(config.active_scene, {})
        if not scene_config:
            raise Exception(f"Scene {config.active_scene} not found in config")
        
        # 场景配置覆盖全局
        if "imu_topic" in scene_config:
            config.imu_topic = scene_config["imu_topic"]
        if "window_size" in scene_config:
            config.window_size = scene_config["window_size"]
        config.anomaly_topic = scene_config.get("anomaly_topic", "")
        config.anomaly_type = scene_config.get("anomaly_type", "")
        config.detect_thresholds = scene_config.get("detect_thresholds", {})
        
        return config

    @staticmethod
    def calculate_variance(data):
        """计算方差"""
        if not data:
            return 0.0
        mean = sum(data) / len(data)
        variance = sum((x - mean)**2 for x in data) / len(data)
        return variance

    def process_imu_data(self, imu_msg):
        """处理单帧IMU数据，返回是否异常"""
        # 缓存IMU数据
        imu_data = {
            "angular_vel_x": imu_msg.angular_velocity.x,
            "angular_vel_y": imu_msg.angular_velocity.y,
            "angular_vel_z": imu_msg.angular_velocity.z,
            "linear_acc_x": imu_msg.linear_acceleration.x,
            "linear_acc_y": imu_msg.linear_acceleration.y,
            "linear_acc_z": imu_msg.linear_acceleration.z,
            "stamp": imu_msg.header.stamp
        }
        self.imu_window.append(imu_data)
        
        # 窗口填满后检测
        if len(self.imu_window) < self.config.window_size:
            return False
        
        # 提取特征
        features = self.extract_features()
        
        # 异常检测
        is_anomaly = self.detect_anomaly(features)
        
        return is_anomaly

    def extract_features(self):
        """场景化特征提取"""
        features = {}
        window = self.imu_window
        
        if self.config.active_scene == "mobile_robot":
            # z轴角速度差值
            ang_z_diff = abs(window[-1]["angular_vel_z"] - window[0]["angular_vel_z"])
            features["angular_z_diff"] = ang_z_diff
            
            # x/y加速度方差
            accel_xy = []
            for data in window:
                accel_xy_mag = math.hypot(data["linear_acc_x"], data["linear_acc_y"])
                accel_xy.append(accel_xy_mag)
            features["accel_xy_var"] = self.calculate_variance(accel_xy)
            
        elif self.config.active_scene == "drone":
            # 三轴角速度方差均值
            ang_x = [d["angular_vel_x"] for d in window]
            ang_y = [d["angular_vel_y"] for d in window]
            ang_z = [d["angular_vel_z"] for d in window]
            
            var_x = self.calculate_variance(ang_x)
            var_y = self.calculate_variance(ang_y)
            var_z = self.calculate_variance(ang_z)
            
            features["angular_var_avg"] = (var_x + var_y + var_z) / 3.0
            
        elif self.config.active_scene == "arm":
            # 加速度峰值
            max_accel = 0.0
            for data in window:
                acc_x_abs = abs(data["linear_acc_x"])
                acc_y_abs = abs(data["linear_acc_y"])
                acc_z_abs = abs(data["linear_acc_z"])
                max_accel = max(max_accel, acc_x_abs, acc_y_abs, acc_z_abs)
            features["accel_peak"] = max_accel
            
        return features

    def detect_anomaly(self, features):
        """场景化异常检测"""
        is_anomaly = False
        thresholds = self.config.detect_thresholds
        
        if self.config.active_scene == "mobile_robot":
            ang_z_diff = features["angular_z_diff"]
            accel_xy_var = features["accel_xy_var"]
            th_ang_z = thresholds["angular_z_diff"]
            th_accel_xy = thresholds["accel_xy_var"]
            
            is_anomaly = (ang_z_diff > th_ang_z) and (accel_xy_var < th_accel_xy)
            
        elif self.config.active_scene == "drone":
            ang_var_avg = features["angular_var_avg"]
            th_ang_var = thresholds["angular_var_avg"]
            
            is_anomaly = (ang_var_avg > th_ang_var)
            
        elif self.config.active_scene == "arm":
            accel_peak = features["accel_peak"]
            th_accel_peak = thresholds["accel_peak"]
            
            is_anomaly = (accel_peak > th_accel_peak)
            
        return is_anomaly