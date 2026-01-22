# 基于参数配置的异常检测功能包 说明文档

# 概述

imu_motion_monitor 基于参数配置的异常检测功能包 是一款轻量化基于参数配置的 ROS IMU 运动异常检测工具包。

工具包适配多场景 IMU 异常检测需求，支持通过修改配置文件一键切换场景，视需求添加场景。具有无侵入、易扩展、低复杂度的特点，可快速集成至任意 ROS 机器人系统。

# 核心特性

- **单文件配置驱动**：所有场景配置整合至一个 YAML 文件，通过 `active_scene` 参数指定当前生效场景，替代多文件方案。

- **无侵入易切换**：仅依赖 ROS 核心包，不修改机器人原有控制逻辑，可即插即用；修改配置文件的 `active_scene` 参数，即可切换检测场景。

- **规则极简可扩展**：异常检测无复杂算法依赖；新增检测场景仅需两步（配置文件加场景块 + 核心逻辑加简单判断），非开发人员也可通过配置调整阈值。

- **双版本兼容**：同时提供 C++（侧重效率）和 Python（侧重调试易用性）双版本实现，功能完全等价，可根据实际需求选择启动。

- **灵活消息输出**：支持两种异常消息类型（`std_msgs/Bool` 简单布尔值 / 自定义 `ImuAnomaly` 消息），可通过配置文件自由切换。

# 依赖环境

## 1. 基础依赖

- ROS 核心包：`roscpp`、`rospy`、`sensor_msgs`、`std_msgs`

- 消息生成依赖：`message_generation`、`message_runtime`

## 2. 第三方依赖

- YAML 解析库（C++ 版本依赖）：`yaml-cpp`

- Python YAML 解析（Python 版本依赖）：`pyyaml`

## 3. 依赖安装命令

```bash

# 1. 安装 ROS 消息生成依赖
sudo apt-get update
sudo apt-get install ros-noetic-message-generation ros-noetic-message-runtime

# 2. 安装 C++ YAML 解析库
sudo apt-get install libyaml-cpp-dev

# 3. 安装 Python YAML 解析库
pip3 install pyyaml

```

# 安装与编译

## 1. 下载与放置

## 2. 编译步骤

```bash

# 进入 ROS 工作空间根目录
cd ~/catkin_ws

# 清理旧编译产物（可选）
catkin_clean -y

# 重新编译
catkin_make -DCMAKE_BUILD_TYPE=Release

# 刷新 ROS 环境
source devel/setup.bash

```


# 配置说明

所有场景的配置均整合在 `config/imu_motion_monitor.yaml` 单个文件中，无需新增/删除文件，仅需修改 `active_scene` 参数即可切换场景。

## 1. 配置文件结构

```yaml

# ===================== 全局通用配置（所有场景共享，可被场景配置覆盖） =====================
global:
  imu_topic: "/imu/data"          # 所有场景默认订阅的 IMU 原始话题（sensor_msgs/Imu 类型）
  window_size: 5                  # 滑动窗口大小（帧），100Hz IMU 对应 50ms，用于特征提取
  anomaly_msg_type: "bool"        # 异常消息类型：bool（std_msgs/Bool）或 custom（自定义 ImuAnomaly）

# ===================== 激活场景指定（修改此处切换场景，无需改代码） =====================
active_scene: "mobile_robot"      # 可选值：mobile_robot（移动机器人打滑）/ drone（无人机抖震）/ arm（机械臂碰撞）

# ===================== 场景专属配置块（新增场景仅需添加此块） =====================
# 场景1：移动机器人 - 轮子打滑检测
mobile_robot:
  imu_topic: "/mobile_robot/imu/data"  # 覆盖全局 IMU 话题（可选，无则沿用 global 配置）
  anomaly_topic: "/mobile_robot/imu_anomaly"  # 异常消息发布话题
  detect_thresholds:                # 该场景专属检测阈值（与核心逻辑一一对应）
    angular_z_diff: 1.0             # z轴角速度帧间突变阈值（rad/s）
    accel_xy_var: 0.1               # x/y轴加速度方差阈值（m²/s⁴）
  detect_rule: "|当前z轴角速度 - 窗口首帧z轴角速度| > angular_z_diff AND x/y轴加速度方差 < accel_xy_var → 异常"
  anomaly_type: "wheel_slip"        # 异常类型标识（自定义消息时生效）

# 场景2：无人机 - 机身抖震检测
drone:
  anomaly_topic: "/drone/imu_anomaly"
  detect_thresholds:
    angular_var_avg: 0.2            # 三轴角速度方差均值阈值（rad²/s²）
  detect_rule: "三轴角速度方差均值 > angular_var_avg → 异常"
  anomaly_type: "body_vibration"

# 场景3：机械臂 - 碰撞冲击检测
arm:
  window_size: 1                    # 覆盖全局窗口大小（碰撞为瞬时事件，仅需1帧）
  anomaly_topic: "/arm/imu_anomaly"
  detect_thresholds:
    accel_peak: 5.0                 # x/y/z轴加速度峰值阈值（m/s²）
  detect_rule: "任一轴加速度绝对值 > accel_peak → 异常"
  anomaly_type: "collision_impact"

```

## 2. 关键配置字段说明

### （1）全局通用配置（global 块）

- `imu_topic`：所有场景默认订阅的 IMU 话题，必须是 `sensor_msgs/Imu` 类型。

- `window_size`：特征提取用的滑动窗口大小（单位：帧），帧频由 IMU 自身决定（如 100Hz IMU，5 帧对应 50ms）。

- `anomaly_msg_type`：异常消息输出类型，二选一：
        

    - `bool`：输出 `std_msgs/Bool`，`data=true` 表示检测到异常，简洁高效。

    - `custom`：输出自定义 `ImuAnomaly`消息，包含时间戳、异常状态、异常类型，信息更完整。

### （2）激活场景指定（active_scene）

指定当前生效的场景，值必须与下方「场景专属配置块的名称」完全一致（如`mobile_robot`、`drone`），修改后重启节点即可生效。

### （3）场景专属配置块（如 mobile_robot 块）

- `imu_topic`（可选）：覆盖全局的 IMU 话题，用于该场景单独订阅不同的 IMU 数据源。

- `window_size`（可选）：覆盖全局的滑动窗口大小，适配不同场景的检测需求（如机械臂碰撞仅需 1 帧）。

- `anomaly_topic`（必选）：该场景异常消息的发布话题，建议按「机器人/场景名称 + /imu_anomaly」命名，避免话题冲突。

- `detect_thresholds`（必选）：该场景的检测阈值，键名需与核心逻辑中的特征名完全一致（如 `angular_z_diff`），值根据实际硬件和场景调整。

- `detect_rule`（注释用）：异常判定规则说明，便于维护和调试，不影响代码运行。

- `anomaly_type`（可选）：异常类型标识，仅在 `anomaly_msg_type: "custom"` 时生效，用于区分不同异常类型。

## 3. 配置修改注意事项

- 所有字段的缩进需严格遵循 YAML 规范

- 修改 `active_scene` 后，必须重启节点才能加载新场景的配置。

- 场景配置块中，未声明的字段（如 `imu_topic`）会自动沿用全局配置。

- 阈值参数需根据实际 IMU 硬件性能、场景需求调整（如不同精度的 IMU，阈值差异较大）。

# 运行说明

支持 C++ 版本（默认，效率高）和 Python 版本（调试方便），可通过修改启动文件切换，无需修改其他配置。

## 1. 启动 C++ 版本

```bash

# 刷新 ROS 环境
source ~/catkin_ws/devel/setup.bash

# 启动核心节点
roslaunch imu_motion_monitor imu_motion_monitor.launch

```

## 2. 启动 Python 版本

修改 `launch/imu_motion_monitor.launch`，注释掉 C++ 节点，启用 Python 节点：

```bash

roslaunch imu_motion_monitor imu_motion_monitor.launch

```

## 3. 运行验证

```bash

# 1. 查看节点是否正常启动
rosnode list | grep imu_motion_monitor

# 2. 查看订阅/发布的话题
rostopic list | grep imu

# 3. 查看异常消息
rostopic echo /mobile_robot/imu_anomaly

```

## 4. 场景切换步骤

1. 停止当前运行的节点（Ctrl + C）。

2. 打开 `config/imu_motion_monitor.yaml`，修改 `active_scene` 的值（如改为 `drone`）。

3. 重新执行启动命令，即可加载新场景的配置，无需重新编译。



# 场景扩展指南

新增一个异常检测场景（如 AGV 偏航异常、机器人跌落异常），仅需 2 步，无需修改核心框架代码。

## 示例：新增「AGV 偏航异常」场景
### 步骤 1：修改配置文件，新增场景块

在 `config/imu_motion_monitor.yaml` 中，新增`agv` 场景块，添加对应配置：

```yaml

# 新增场景4：AGV 偏航异常（z轴角速度方差过大）
agv:
  imu_topic: "/agv/imu/data"          # AGV 专属 IMU 话题
  anomaly_topic: "/agv/imu_anomaly"   # AGV 异常发布话题
  detect_thresholds:
    angular_z_var: 0.5                # z轴角速度方差阈值（rad²/s²）
  detect_rule: "z轴角速度方差 > angular_z_var → 偏航异常"
  anomaly_type: "yaw_abnormal"        # 异常类型标识

```

### 步骤 2：修改核心逻辑，新增场景检测代码

分别修改 C++ 和 Python 版本的核心逻辑，新增「特征提取」和「异常检测」的简单判断（复用现有框架）。



### 步骤 3：生效新增场景

1. 若修改了 C++ 代码，需重新编译：
`catkin_make -DCMAKE_BUILD_TYPE=Release`。

2. 修改配置文件 `active_scene: "agv"`。

3. 重启节点，即可启用 AGV 偏航异常检测功能。

# 故障排查指南

## 常见问题及解决方案

|问题现象|可能原因|解决方案|
|---|---|---|
|编译报错：fatal error: imu_motion_monitor/ImuAnomaly.h: 没有那个文件或目录|1. 未安装消息生成依赖；2. CMakeLists.txt 未配置消息生成规则；3. 未重新编译。|1. 安装 `message_generation`、`message_runtime`；2. 检查 CMakeLists.txt 中 `add_message_files`、`generate_messages` 配置；3. 执行 `catkin_clean -y` 后重新编译。|
|编译报错：undefined reference to yaml-cpp 相关函数|1. 未安装 `libyaml-cpp-dev`；2. CMakeLists.txt 未显式链接 yaml-cpp 库。|1. 执行 `sudo apt-get install libyaml-cpp-dev`；2. 检查 CMakeLists.txt 中 `find_package(yaml-cpp REQUIRED)` 和 `target_link_libraries` 配置。|
|节点启动失败，提示「Config file not found」|配置文件路径错误，节点未找到 `imu_motion_monitor.yaml`。|1. 检查配置文件是否在 `config/` 目录下；2. 确认启动命令执行前已刷新 ROS 环境（`source devel/setup.bash`）。|
|节点启动成功，但无异常消息发布|1. IMU 话题无数据；2. 滑动窗口未填满；3. 阈值设置不合理，未检测到异常；4. 场景配置错误。|1. 用 `rostopic echo` 检查 IMU 话题是否有数据；2. 增大 IMU 数据发送频率或调整 `window_size`；3. 调整 `detect_thresholds` 阈值；4. 检查 `active_scene` 是否与场景块名称一致。|

