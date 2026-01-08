#!/usr/bin/env python3
import rospy
import rosbag
import numpy as np
import scipy.fftpack
import scipy.signal
import os
import yaml
from sensor_msgs.msg import Imu, MagneticField

class ImuAnalyzerConfig:
    def __init__(self):
        # 基础配置
        self.mode = "offline"
        self.imu_topic = "/imu/data_raw"
        self.mag_topic = "/imu/mag"
        self.use_mag = False
        self.sample_rate = 100.0
        
        # 离线配置
        self.bag_path = ""
        self.start_time = 0.0
        self.end_time = 0.0
        
        # 预处理配置
        self.preprocess = {
            "enable": False,
            "sigma": 3.0,
            "lowpass_cutoff": 10.0,
            "resample_rate": 100.0
        }
        
        # 频域分析配置
        self.freq_analysis = {
            "enable": True,
            "fft_size": 1024,
            "window": "hann"
        }
        
        # Allan分析配置
        self.allan_analysis = {
            "enable": True,
            "max_tau": 100.0,
            "tau_num": 100
        }
        
        # 输出配置
        self.output = {
            "dir": "./imu_analysis_results",
            "plot": True
        }
        
        # 在线扩展配置
        self.collect_duration = 0.0

class OnlineImuSubscriber:
    def __init__(self, config):
        self.config = config
        self.imu_data = []
        self.mag_data = []
        self.is_running = False
        
        self.imu_sub = rospy.Subscriber(config.imu_topic, Imu, self.imu_callback)
        if config.use_mag:
            self.mag_sub = rospy.Subscriber(config.mag_topic, MagneticField, self.mag_callback)
        
        rospy.loginfo(f"在线模式：订阅IMU话题 {config.imu_topic}")
        if config.use_mag:
            rospy.loginfo(f"在线模式：订阅磁力计话题 {config.mag_topic}")

    def imu_callback(self, msg):
        if not self.is_running:
            return
        data = {
            "timestamp": msg.header.stamp.to_sec(),
            "accel": [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            "gyro": [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            "mag": [0.0, 0.0, 0.0],
            "has_mag": False
        }
        self.imu_data.append(data)

    def mag_callback(self, msg):
        if not self.is_running:
            return
        data = {
            "timestamp": msg.header.stamp.to_sec(),
            "mag": [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z]
        }
        self.mag_data.append(data)

    def get_data(self, collect_duration=0.0):
        self.is_running = True
        self.imu_data.clear()
        self.mag_data.clear()
        
        if collect_duration > 0:
            rospy.loginfo(f"在线采集数据中...（时长：{collect_duration:.2f}s）")
            start_time = rospy.Time.now()
            while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < collect_duration:
                rospy.sleep(0.01)
        else:
            rospy.loginfo("在线采集数据中...（按Ctrl+C停止）")
            while not rospy.is_shutdown() and self.is_running:
                rospy.sleep(0.1)
        
        self.is_running = False
        
        if self.config.use_mag and self.mag_data:
            for imu_d in self.imu_data:
                if self.mag_data:
                    mag_d = min(self.mag_data, key=lambda x: abs(x["timestamp"] - imu_d["timestamp"]))
                    if abs(mag_d["timestamp"] - imu_d["timestamp"]) < 0.01:
                        imu_d["mag"] = mag_d["mag"]
                        imu_d["has_mag"] = True
        
        rospy.loginfo(f"在线采集完成：{len(self.imu_data)}个IMU数据点")
        return self.imu_data

    def stop(self):
        self.is_running = False

# 工具函数
def create_dir(dir_path):
    if not os.path.exists(dir_path):
        os.makedirs(dir_path, exist_ok=True)
    return dir_path

def load_config(config_path):
    config = ImuAnalyzerConfig()
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            yaml_config = yaml.safe_load(f)["imu_analyzer"]
            # 基础配置
            if "mode" in yaml_config: config.mode = yaml_config["mode"]
            if "imu_topic" in yaml_config: config.imu_topic = yaml_config["imu_topic"]
            if "mag_topic" in yaml_config: config.mag_topic = yaml_config["mag_topic"]
            if "use_mag" in yaml_config: config.use_mag = yaml_config["use_mag"]
            if "sample_rate" in yaml_config: config.sample_rate = yaml_config["sample_rate"]
            
            # 离线配置
            if "bag_path" in yaml_config: config.bag_path = yaml_config["bag_path"]
            if "start_time" in yaml_config: config.start_time = yaml_config["start_time"]
            if "end_time" in yaml_config: config.end_time = yaml_config["end_time"]
            
            # 预处理配置
            if "preprocess" in yaml_config:
                if "enable" in yaml_config["preprocess"]: config.preprocess["enable"] = yaml_config["preprocess"]["enable"]
                if "sigma" in yaml_config["preprocess"]: config.preprocess["sigma"] = yaml_config["preprocess"]["sigma"]
                if "lowpass_cutoff" in yaml_config["preprocess"]: config.preprocess["lowpass_cutoff"] = yaml_config["preprocess"]["lowpass_cutoff"]
                if "resample_rate" in yaml_config["preprocess"]: config.preprocess["resample_rate"] = yaml_config["preprocess"]["resample_rate"]
            
            # 频域分析配置
            if "freq_analysis" in yaml_config:
                if "enable" in yaml_config["freq_analysis"]: config.freq_analysis["enable"] = yaml_config["freq_analysis"]["enable"]
                if "fft_size" in yaml_config["freq_analysis"]: config.freq_analysis["fft_size"] = yaml_config["freq_analysis"]["fft_size"]
                if "window" in yaml_config["freq_analysis"]: config.freq_analysis["window"] = yaml_config["freq_analysis"]["window"]
            
            # Allan分析配置
            if "allan_analysis" in yaml_config:
                if "enable" in yaml_config["allan_analysis"]: config.allan_analysis["enable"] = yaml_config["allan_analysis"]["enable"]
                if "max_tau" in yaml_config["allan_analysis"]: config.allan_analysis["max_tau"] = yaml_config["allan_analysis"]["max_tau"]
                if "tau_num" in yaml_config["allan_analysis"]: config.allan_analysis["tau_num"] = yaml_config["allan_analysis"]["tau_num"]
            
            # 输出配置
            if "output" in yaml_config:
                if "dir" in yaml_config["output"]: config.output["dir"] = yaml_config["output"]["dir"]
                if "plot" in yaml_config["output"]: config.output["plot"] = yaml_config["output"]["plot"]
            
            # 在线扩展配置
            if "collect_duration" in yaml_config: config.collect_duration = yaml_config["collect_duration"]
    return config

def load_ros_params(nh):
    config = ImuAnalyzerConfig()
    # 基础配置
    config.mode = nh.get_param("imu_analyzer/mode", config.mode)
    config.imu_topic = nh.get_param("imu_analyzer/imu_topic", config.imu_topic)
    config.mag_topic = nh.get_param("imu_analyzer/mag_topic", config.mag_topic)
    config.use_mag = nh.get_param("imu_analyzer/use_mag", config.use_mag)
    config.sample_rate = nh.get_param("imu_analyzer/sample_rate", config.sample_rate)
    
    # 离线配置
    config.bag_path = nh.get_param("imu_analyzer/bag_path", config.bag_path)
    config.start_time = nh.get_param("imu_analyzer/start_time", config.start_time)
    config.end_time = nh.get_param("imu_analyzer/end_time", config.end_time)
    
    # 预处理配置
    config.preprocess["enable"] = nh.get_param("imu_analyzer/preprocess/enable", config.preprocess["enable"])
    config.preprocess["sigma"] = nh.get_param("imu_analyzer/preprocess/sigma", config.preprocess["sigma"])
    config.preprocess["lowpass_cutoff"] = nh.get_param("imu_analyzer/preprocess/lowpass_cutoff", config.preprocess["lowpass_cutoff"])
    config.preprocess["resample_rate"] = nh.get_param("imu_analyzer/preprocess/resample_rate", config.preprocess["resample_rate"])
    
    # 频域分析配置
    config.freq_analysis["enable"] = nh.get_param("imu_analyzer/freq_analysis/enable", config.freq_analysis["enable"])
    config.freq_analysis["fft_size"] = nh.get_param("imu_analyzer/freq_analysis/fft_size", config.freq_analysis["fft_size"])
    config.freq_analysis["window"] = nh.get_param("imu_analyzer/freq_analysis/window", config.freq_analysis["window"])
    
    # Allan分析配置
    config.allan_analysis["enable"] = nh.get_param("imu_analyzer/allan_analysis/enable", config.allan_analysis["enable"])
    config.allan_analysis["max_tau"] = nh.get_param("imu_analyzer/allan_analysis/max_tau", config.allan_analysis["max_tau"])
    config.allan_analysis["tau_num"] = nh.get_param("imu_analyzer/allan_analysis/tau_num", config.allan_analysis["tau_num"])
    
    # 输出配置
    config.output["dir"] = nh.get_param("imu_analyzer/output/dir", config.output["dir"])
    config.output["plot"] = nh.get_param("imu_analyzer/output/plot", config.output["plot"])
    
    # 在线扩展配置
    config.collect_duration = nh.get_param("imu_analyzer/collect_duration", config.collect_duration)
    
    return config

def load_from_bag(config):
    imu_data = []
    mag_data = []
    
    try:
        bag = rosbag.Bag(config.bag_path, 'r')
    except Exception as e:
        rospy.logerr(f"打开Bag失败: {str(e)}")
        return imu_data
    
    start_time = rospy.Time(config.start_time) if config.start_time > 0 else None
    end_time = rospy.Time(config.end_time) if config.end_time > 0 else None
    
    for topic, msg, t in bag.read_messages(topics=[config.imu_topic], start_time=start_time, end_time=end_time):
        if isinstance(msg, Imu):
            data = {
                "timestamp": t.to_sec(),
                "accel": [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                "gyro": [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                "mag": [0.0, 0.0, 0.0],
                "has_mag": False
            }
            imu_data.append(data)
    
    if config.use_mag:
        for topic, msg, t in bag.read_messages(topics=[config.mag_topic], start_time=start_time, end_time=end_time):
            if isinstance(msg, MagneticField):
                data = {
                    "timestamp": t.to_sec(),
                    "mag": [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z]
                }
                mag_data.append(data)
    
    if config.use_mag and mag_data:
        for imu_d in imu_data:
            mag_d = min(mag_data, key=lambda x: abs(x["timestamp"] - imu_d["timestamp"]))
            if abs(mag_d["timestamp"] - imu_d["timestamp"]) < 0.01:
                imu_d["mag"] = mag_d["mag"]
                imu_d["has_mag"] = True
    
    bag.close()
    rospy.loginfo(f"离线加载IMU数据: {len(imu_data)}个点")
    return imu_data

def filter_outliers(imu_data, sigma=3.0):
    if not imu_data:
        return imu_data
    
    accel = np.array([d["accel"] for d in imu_data])
    gyro = np.array([d["gyro"] for d in imu_data])
    
    accel_mean = np.mean(accel, axis=0)
    accel_std = np.std(accel, axis=0)
    gyro_mean = np.mean(gyro, axis=0)
    gyro_std = np.std(gyro, axis=0)
    
    filtered = []
    for i, d in enumerate(imu_data):
        accel_ok = np.all(np.abs(accel[i] - accel_mean) < sigma * accel_std)
        gyro_ok = np.all(np.abs(gyro[i] - gyro_mean) < sigma * gyro_std)
        if accel_ok and gyro_ok:
            filtered.append(d)
    
    removed = len(imu_data) - len(filtered)
    rospy.loginfo(f"异常值过滤：移除{removed}个点，剩余{len(filtered)}个点")
    return filtered

def freq_analysis(imu_data, config):
    if not imu_data or not config.freq_analysis["enable"]:
        return
    
    create_dir(config.output["dir"])
    fs = config.preprocess["resample_rate"]
    n = config.freq_analysis["fft_size"]
    
    accel = np.array([d["accel"] for d in imu_data])
    gyro = np.array([d["gyro"] for d in imu_data])
    
    axes = ['x', 'y', 'z']
    for i, ax in enumerate(axes):
        # 加速度计PSD
        data = accel[:, i]
        data = np.pad(data, (0, n - len(data) % n), mode='constant')[:n]
        
        if config.freq_analysis["window"] == 'hann':
            window = np.hanning(n)
            data = data * window
        
        fft_result = scipy.fftpack.fft(data)
        psd = np.abs(fft_result[:n//2+1])**2 / (fs * n)
        
        freq = scipy.fftpack.fftfreq(n, 1/fs)[:n//2+1]
        np.savetxt(
            os.path.join(config.output["dir"], f'accel_{ax}_psd.csv'),
            np.column_stack((freq, psd)),
            header='freq,psd', comments='', delimiter=','
        )
        
        # 陀螺仪PSD
        data = gyro[:, i]
        data = np.pad(data, (0, n - len(data) % n), mode='constant')[:n]
        if config.freq_analysis["window"] == 'hann':
            data = data * window
        
        fft_result = scipy.fftpack.fft(data)
        psd = np.abs(fft_result[:n//2+1])**2 / (fs * n)
        
        np.savetxt(
            os.path.join(config.output["dir"], f'gyro_{ax}_psd.csv'),
            np.column_stack((freq, psd)),
            header='freq,psd', comments='', delimiter=','
        )
    
    rospy.loginfo(f"频域分析完成：PSD文件已保存到 {config.output['dir']}")

def allan_analysis(imu_data, config):
    if not imu_data or not config.allan_analysis["enable"]:
        return
    
    create_dir(config.output["dir"])
    dt = 1.0 / config.preprocess["resample_rate"]
    max_tau = config.allan_analysis["max_tau"]
    tau_num = config.allan_analysis["tau_num"]
    
    taus = np.logspace(np.log10(dt), np.log10(max_tau), tau_num)
    
    accel = np.array([d["accel"] for d in imu_data])
    gyro = np.array([d["gyro"] for d in imu_data])
    
    axes = ['x', 'y', 'z']
    for i, ax in enumerate(axes):
        # 加速度计Allan方差
        data = accel[:, i]
        allan_var = np.zeros_like(taus)
        for j, tau in enumerate(taus):
            m = int(round(tau / dt))
            if m < 1 or m >= len(data):
                continue
            n = len(data) - 2*m
            if n <= 0:
                continue
            diff = np.sum(np.reshape(data[:2*m*n], (n, 2, m)), axis=2)
            allan_var[j] = np.mean((diff[:, 1] - diff[:, 0])**2) / (2 * m**2)
        
        allan_dev = np.sqrt(allan_var)
        np.savetxt(
            os.path.join(config.output["dir"], f'accel_{ax}_allan.csv'),
            np.column_stack((taus, allan_var, allan_dev)),
            header='tau,allan_var,allan_deviation', comments='', delimiter=','
        )
        
        # 陀螺仪Allan方差
        data = gyro[:, i]
        allan_var = np.zeros_like(taus)
        for j, tau in enumerate(taus):
            m = int(round(tau / dt))
            if m < 1 or m >= len(data):
                continue
            n = len(data) - 2*m
            if n <= 0:
                continue
            diff = np.sum(np.reshape(data[:2*m*n], (n, 2, m)), axis=2)
            allan_var[j] = np.mean((diff[:, 1] - diff[:, 0])**2) / (2 * m**2)
        
        allan_dev = np.sqrt(allan_var)
        np.savetxt(
            os.path.join(config.output["dir"], f'gyro_{ax}_allan.csv'),
            np.column_stack((taus, allan_var, allan_dev)),
            header='tau,allan_var,allan_deviation', comments='', delimiter=','
        )
    
    rospy.loginfo(f"Allan方差分析完成：结果已保存到 {config.output['dir']}")