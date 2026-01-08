# imu_freq_noise_analyzer ROS包使用指南
用于 IMU（惯性测量单元）的频率特性分析和噪声特性标定，支持 6 轴 / 9 轴 IMU，提供 C++/Python 双版本实现，支持在线 / 离线两种运行模式，输出频域分析（PSD）和 Allan 方差分析结果，可直接用于 IMU 性能评估与标定。

## 一、包介绍
### 核心功能
1. 3σ 异常值过滤（数据预处理）
2. 功率谱密度（PSD）频域分析
3. Allan 方差分析（噪声标定）

## 二、环境依赖
### 通用依赖
```bash
# ROS核心依赖
sudo apt-get install ros-noetic-roscpp ros-noetic-rospy ros-noetic-rosbag ros-noetic-sensor-msgs

# C++依赖
sudo apt-get install libeigen3-dev libyaml-cpp-dev libfftw3-dev

# Python依赖
sudo apt-get install python3-numpy python3-scipy python3-yaml python3-matplotlib

```

## 三、编译与安装
1. 将包放入 ROS 工作空间的src目录
2. 编译
```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release  # Release模式编译，提升计算效率
```
3. 刷新环境
```bash
source devel/setup.bash
```

## 四、快速使用

1. 方式 1：启动文件一键运行（推荐）
```bash
# C++离线模式（解析Bag文件）
roslaunch imu_freq_noise_analyzer imu_analyzer.launch lang:=cpp mode:=offline

# Python在线模式（实时采集10秒IMU数据）
roslaunch imu_freq_noise_analyzer imu_analyzer.launch lang:=python mode:=online

# 自定义参数运行（如指定在线采集时长20秒）
roslaunch imu_freq_noise_analyzer imu_analyzer.launch lang:=cpp mode:=online imu_analyzer/collect_duration:=20.0
```
2. 方式 2：直接运行节点
```bash
# C++节点（指定配置文件）
rosrun imu_freq_noise_analyzer imu_freq_noise_analyzer_node $(rospack find imu_freq_noise_analyzer)/config/imu_analyzer_config.yaml

# Python节点（指定配置文件）
rosrun imu_freq_noise_analyzer imu_analyzer_node.py $(rospack find imu_freq_noise_analyzer)/config/imu_analyzer_config.yaml

```
## 五、配置说明
所有核心参数通过 config/imu_analyzer_config.yaml 统一配置，C++/Python 版本共用，支持通过 ROS 参数覆盖。

```bash
imu_analyzer:
  # 基础配置
  mode: "offline"                  # 运行模式：offline/online
  imu_topic: "/imu/data_raw"       # IMU数据话题（需与实际硬件匹配）
  mag_topic: "/imu/mag"            # 磁力计话题（9轴IMU启用）
  use_mag: false                   # 是否使用磁力计数据
  sample_rate: 100.0               # IMU原始采样率(Hz)
  
  # 离线模式专属
  bag_path: "$(find imu_freq_noise_analyzer)/data/imu_test.bag"  # Bag文件路径
  start_time: 0.0                  # 解析起始时间（0=从开头）
  end_time: 0.0                    # 解析结束时间（0=到结尾）
  
  # 数据预处理
  preprocess:
    enable: true                   # 启用3σ异常值过滤
    sigma: 3.0                     # 过滤系数（3σ原则）
    lowpass_cutoff: 10.0           # 低通截止频率（预留扩展）
    resample_rate: 100.0           # 重采样率（分析用）
  
  # 频域分析（PSD）
  freq_analysis:
    enable: true                   # 启用频域分析
    fft_size: 1024                 # FFT计算长度
    window: "hann"                 # 窗函数：hann/hamming/rect
  
  # Allan方差分析（噪声标定）
  allan_analysis:
    enable: true                   # 启用Allan方差分析
    max_tau: 100.0                 # 最大tau值(秒)
    tau_num: 100                   # tau点数（分析精度）
  
  # 输出配置
  output:
    dir: "$(find imu_freq_noise_analyzer)/results"  # 结果保存目录
    plot: true                     # Python版本支持绘图（C++版本仅输出数据）
  
  # 在线模式专属
  collect_duration: 0.0            # 采集时长（0=无限采集，按Ctrl+C停止）
```

配置修改建议
  1. 离线模式：修改 bag_path 为实际的 IMU Bag 文件路径，确保 imu_topic 与 Bag 内话题一致；
  2. 在线模式：修改 imu_topic 为实际硬件发布的 IMU 话题，设置 collect_duration 指定采集时长；
  3. 9 轴 IMU：将 use_mag 设为 true，并修改 mag_topic 为磁力计话题；
  4. 精度调整：频域分析可增大 fft_size 提升频率分辨率，Allan 分析可增大 tau_num 提升精度。

## 六、输出结果

分析结果默认保存到 imu_freq_noise_analyzer/results 目录，包含以下 CSV 文件：
| 文件名格式 | 说明 |列名 |
|-------|-------|-------|
| accel_x_psd.csv | X 轴加速度计 PSD 数据| freq（频率）, psd（功率谱密度） |
| gyro_y_psd.csv | Y 轴陀螺仪 PSD 数据 | freq, psd |
| accel_z_allan.csv |Z 轴加速度计 Allan 方差数据 | tau（时间间隔）, allan_var（方差）, allan_deviation（标准差）|
| gyro_x_allan.csv | X 轴陀螺仪 Allan 方差数据 |tau, allan_var, allan_deviation定 |



## 七、常见问题

1. 编译报错 "fatal error: imu_freq_noise_analyzer/imu_analyzer_core.h: No such file or directory"
```bash
解决：检查 CMakeLists.txt 中的 include_directories 是否包含 include 目录，确保头文件路径正确；编译前执行 catkin_make clean 清理缓存后重新编译。
```
2. 在线模式无法采集数据
```bash
解决：
检查 imu_topic 是否与实际硬件发布的话题一致（使用 rostopic list 确认）；
确保 IMU 硬件已正常启动并发布数据（使用 rostopic echo /imu/data_raw 验证）；
检查 ROS 节点是否正常（使用 rosnode list 确认节点已启动）。
```
3. 离线模式提示 "打开 Bag 失败"
```bash
解决：
检查 bag_path 是否为绝对路径，或使用 $(find imu_freq_noise_analyzer) 拼接相对路径；
确保 Bag 文件存在且有权限访问（执行 ls -l <bag_path> 验证）；
检查 Bag 内是否包含指定的 imu_topic（使用 rosbag info <bag_path> 查看）。
```

4. Allan 方差分析结果为空
```bash
解决：
确保 IMU 数据量足够（建议至少采集 10 分钟离线数据，或在线采集≥60 秒）；
检查 resample_rate 是否与实际采样率匹配；
减小 max_tau 或增大数据量，确保 tau 范围内有足够的有效数据。
```