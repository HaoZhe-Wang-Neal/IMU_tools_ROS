# VQF ROS包使用指南
轻量级VQF（Vector-Quest Filter）姿态解算ROS工具包，支持6轴/9轴IMU实时姿态估计，提供C++/Python双版本实现

## 一、包介绍
### 核心功能
- 基于VQF算法的IMU姿态解算（四元数/RPY输出）
- 支持6轴（加速度+角速度）/9轴（新增磁力计）IMU适配
- 可配置的比例/积分增益，适配不同IMU硬件特性
- 线程安全的数据回调与姿态更新

## 二、环境依赖
### 通用依赖
  ROS (Kinetic/Melodic/Noetic)
  Eigen3 (C++ 版本)
  Python3-numpy (Python 版本)

## 三、快速开始
1. 编译
```bash
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```
2. 启动
```bash
# C++版本
roslaunch vqf vqf.launch is_9axis:=true

# Python版本（修改launch文件注释，启用Python节点）
roslaunch vqf vqf.launch is_9axis:=false
```

## 四、关键话题
| 话题名 | 类型 | 说明 |
|-------|-------|-------|
| /imu/cleaned | sensor_msgs/Imu | 输入清洗后的 IMU 数据 |
| /imu/mag | sensor_msgs/MagneticField | 磁力计数据（9 轴启用） |
| /imu/vqf/pose | geometry_msgs/Quaternion | 输出姿态四元数 |
| /imu/vqf/rpy | geometry_msgs/Vector3 | 输出 RPY 姿态（弧度） |

## 五、算法原理

VQF 是一种基于四元数的轻量级姿态估计算法，核心分为两步：
1. 预测步：通过陀螺仪积分更新姿态；
2. 校正步：通过加速度计（重力向量）/ 磁力计（地磁向量）校正姿态误差，结合 PI 控制器抑制漂移。


## 六、核心参数说明


| 参数名 | 类型 | 说明 |
|-------|-------|-------|
| is_9axis | bool| 是否启用 9 轴 IMU（磁力计） |
| kp_acc | double | 加速度计比例增益 |
| ki_acc | double | 加速度计积分增益 |
| kp_mag | double |磁力计比例增益（9 轴）|
| ki_mag | double | 磁力计积分增益（9 轴）|
| sample_freq	double | double| IMU 采样频率 (Hz) |
| publish_rate | double| 姿态发布频率 (Hz) |

