# imu_clean ROS包使用指南
该包提供IMU数据清洗功能，支持**C++/Python双版本**，适配6/9轴IMU，实现基于滑动窗口3σ的野值检测+前一帧填充，参数可动态配置。

## 一、包介绍
### 核心功能
1. 支持6轴（加速度+角速度）/9轴（+磁力计）IMU数据处理；
2. 野值检测：滑动窗口3σ统计阈值 + 绝对阈值；
3. 野值填充：检测到野值后自动用前一帧有效数据填充；
4. 动态参数配置：支持ROS参数服务器/代码内默认值两种方式。

## 二、环境依赖
### 通用依赖
```bash
# ROS基础依赖（以noetic为例）
sudo apt install ros-noetic-roscpp ros-noetic-rospy ros-noetic-sensor-msgs
# C++基础依赖
sudo apt install libeigen3-dev ros-noetic-eigen-conversions
#Python 额外依赖
sudo apt install python3-numpy

```

## 三、编译与安装
1. 将imu_clean包放入 ROS 工作空间的src目录
2. 编译
```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```
3. 刷新环境
```bash
source devel/setup.bash
```

## 四、C++ 代码使用

1. 运行节点
```bash
# 默认6轴配置
rosrun imu_clean imu_clean_node_cpp

# 动态配置参数（9轴+滑动窗口30）
rosrun imu_clean imu_clean_node_cpp _is_9axis:=true _sliding_window_size:=30
```
2. 参数配置
```bash
#方式 1：命令行动态传参
rosrun imu_clean imu_clean_node_cpp \
  _is_9axis:=true \          # 9轴模式
  _sliding_window_size:=25 \ # 滑动窗口大小
  _sigma_multiplier:=2.5 \   # 3σ倍数改为2.5
  _accel_abs_thresh:=15.0    # 加速度绝对阈值15m/s²

#方式 2：ROS 参数服务器，设置参数后启动节点
rosparam set /imu_clean_node_cpp/is_9axis true
rosrun imu_clean imu_clean_node_cpp

```
3. 话题说明

| 订阅话题 | 发布话题 | 说明 |
|-------|-------|-------|
| /imu/data_raw | /imu/data_cleaned | 6/9 轴 IMU 清洗后数据 |
| /imu/mag（9 轴） | /imu/mag_cleaned | 9 轴磁力计清洗后数据 |


## 五、Python  代码使用

1. 运行节点
```bash
# 默认6轴配置
rosrun imu_clean imu_clean_node_py

# 动态配置9轴
rosrun imu_clean imu_clean_node_py _is_9axis:=true
```
2. 参数配置
```bash
#方式 1：命令行动态传参
rosrun imu_clean imu_clean_node_py \
  _is_9axis:=true \
  _sliding_window_size:=20 \
  _gyro_abs_thresh:=4.0

#方式 2：ROS 参数服务器，设置参数后启动节点
rosparam set /imu_clean_node_py/is_9axis true
rosrun imu_clean imu_clean_node_py

```
3. 话题说明

| 订阅话题 | 发布话题 | 说明 |
|-------|-------|-------|
| /imu/data_raw | /imu/data_cleaned | 6/9 轴 IMU 清洗后数据 |
| /imu/mag（9 轴） | /imu/mag_cleaned | 9 轴磁力计清洗后数据 |


## 六、核心参数说明


| 参数名 | 作用 | 建议值 |
|-------|-------|-------|
| is_9axis | 是否启用 9 轴模式| false（6 轴）/true（9 轴） |
| sliding_window_size | 滑动窗口大小 | 10-30（小窗口响应快，大窗口统计稳） |
| sigma_multiplier | 3σ 倍数 | 2.5-3.0 |
| accel_abs_thresh | /加速度绝对阈值（m/s²） |视使用环境确定 |
| gyro_abs_thresh | 角速度绝对阈值（rad/s） | 视使用环境确定|
| mag_abs_thresh | 磁力计绝对阈值（uT）| 视使用环境确定 |



## 七、常见问题

1. 编译报错：Eigen 找不到
```bash
确认安装libeigen3-dev，并在CMakeLists.txt中正确引入EIGEN3
```
2. Python 节点提示找不到模块
```bash
确保编译后执行source devel/setup.bash
```
3. 9 轴模式下磁力计无数据
```bash
检查磁力计订阅话题是否正确（默认/imu/mag），确认硬件已发布磁力计数据
```
