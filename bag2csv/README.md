# bag2csv ROS包使用指南
该包提供ROS bag文件数据转换功能，支持将任意话题数据（尤其是IMU、磁力计等传感器数据）从.bag文件转换为CSV格式。核心优势：通用性强、配置灵活、支持任意ROS消息类型。

## 一、包介绍
### 核心功能
1. 通用数据转换：支持任意ROS消息类型（IMU、磁力计、Odometry、自定义消息等）
2. 自动字段解析：递归解析嵌套消息结构（如header.stamp.secs → header.stamp.secs）
3. 灵活配置：
        指定目标话题（默认转换所有话题）
        选择时间格式（秒/纳秒/毫秒）
        控制CSV头部（包含/不包含字段名）

## 二、环境依赖
### 通用依赖
```bash
# ROS基础依赖
sudo apt install ros-noetic-roscpp ros-noetic-rospy ros-noetic-sensor-msgs ros-noetic-rosbag
# Python依赖
sudo apt install python3-pip
pip3 install rospkg csv
#C++版本无需额外依赖
```

## 三、编译与安装
1. 将包放入ROS工作空间
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


```bash
#IMU数据转换（C++版）
rosrun bag2csv bag2csv_node_cpp \
  _input_bag:=/path/to/data.bag \
  _output_dir:=/path/to/output
  #C++版本可以处理处理sensor_msgs/Imu和MagneticField
```

## 五、Python  代码使用

```bash
# 基础用法：转换所有话题到CSV
rosrun bag2csv bag2csv_node_py _input_bag:=/path/to/data.bag _output_dir:=/path/to/output

# 指定目标话题（仅转换IMU数据）
rosrun bag2csv bag2csv_node_py \
  _input_bag:=/path/to/data.bag \
  _output_dir:=/path/to/output \
  _target_topics:=["/imu/data"]  # 注意双引号和数组格式

# 时间格式为纳秒（默认秒）
rosrun bag2csv bag2csv_node_py \
  _time_format:=ns \
  _input_bag:=/path/to/data.bag
```

## 六、核心参数说明


| 参数名 | 作用 | 默认值 |
|-------|-------|-------|
| input_bag | 输入bag文件路径| false（6 轴）/true（9 轴） |
| output_dir | 输出CSV目录 | 10-30（小窗口响应快，大窗口统计稳） |
| target_topics | 目标话题列表 | 2.5-3.0 |
| include_header | 是否包含CSV头部 |视使用环境确定 |
| time_format | 时间格式 | 视使用环境确定|
| verbose | 显示详细日志| 视使用环境确定 |


## 七、常见问题

1. C++版本无输出
```bash
#C++版仅处理sensor_msgs/Imu和sensor_msgs/MagneticField
# 确认bag中话题类型
rosbag info /path/to/data.bag
# 当话题为IMU/MagneticField时使用C++版
# 否则改用Python版（通用）
rosrun bag2csv bag2csv_node_py _input_bag:=/path/to/data.bag
```
2. CSV字段名包含特殊字符
```bash
原因：消息字段名含/或.（如header.stamp）
解决：自动处理为header.stamp → header.stamp（CSV列名已安全处理）
```


