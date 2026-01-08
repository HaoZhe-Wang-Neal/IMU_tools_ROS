#!/usr/bin/env python3
import rospy
import sys
import os
from imu_analyzer_core import (
    ImuAnalyzerConfig, OnlineImuSubscriber,
    load_config, load_ros_params, load_from_bag,
    filter_outliers, freq_analysis, allan_analysis, create_dir
)

class ImuFreqNoiseAnalyzerNode:
    def __init__(self, argc, argv):
        rospy.init_node('imu_freq_noise_analyzer_node', anonymous=True)
        self.nh = rospy.get_param_namespace()
        self.argc = argc
        self.argv = argv
        self.config = ImuAnalyzerConfig()
        self.subscriber = None
        
        # 注册退出回调
        rospy.on_shutdown(self.shutdown_hook)

    def load_config(self):
        if self.argc >= 2 and not self.argv[1].startswith('_'):
            config_path = self.argv[1]
            if os.path.exists(config_path):
                self.config = load_config(config_path)
                rospy.loginfo(f"从配置文件加载参数：{config_path}")
            else:
                rospy.logerr(f"配置文件不存在：{config_path}")
                sys.exit(1)
        else:
            self.config = load_ros_params(rospy)
            rospy.loginfo("从ROS参数服务器加载参数")

    def validate_config(self):
        if self.config.mode not in ['offline', 'online']:
            rospy.logerr(f"无效模式：{self.config.mode}（仅支持offline/online）")
            return False
        if self.config.mode == 'offline' and not self.config.bag_path:
            rospy.logerr("离线模式必须指定bag_path")
            return False
        return True

    def collect_imu_data(self):
        imu_data = []
        if self.config.mode == 'offline':
            imu_data = load_from_bag(self.config)
        elif self.config.mode == 'online':
            self.subscriber = OnlineImuSubscriber(self.config)
            collect_duration = self.config.collect_duration
            collect_duration = rospy.get_param("imu_analyzer/collect_duration", collect_duration)
            imu_data = self.subscriber.get_data(collect_duration)
        return imu_data

    def shutdown_hook(self):
        if self.subscriber:
            self.subscriber.stop()
        rospy.loginfo("IMU分析节点已退出")

    def run(self):
        self.load_config()
        if not self.validate_config():
            sys.exit(1)
        
        imu_data = self.collect_imu_data()
        if not imu_data:
            rospy.logerr("未获取到有效IMU数据")
            sys.exit(1)
        
        if self.config.preprocess["enable"]:
            imu_data = filter_outliers(imu_data, self.config.preprocess["sigma"])
        
        freq_analysis(imu_data, self.config)
        allan_analysis(imu_data, self.config)
        
        rospy.loginfo("="*50)
        rospy.loginfo("IMU频率/噪声分析完成！")
        rospy.loginfo(f"结果保存目录：{self.config.output['dir']}")
        rospy.loginfo("="*50)

if __name__ == '__main__':
    try:
        node = ImuFreqNoiseAnalyzerNode(len(sys.argv), sys.argv)
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断")
    except Exception as e:
        rospy.logerr(f"运行出错: {str(e)}")
        sys.exit(1)