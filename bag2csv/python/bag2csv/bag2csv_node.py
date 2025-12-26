#!/usr/bin/env python3
import rospy
import rosbag
import csv
import os
from os.path import join, exists, basename
from bag2csv_config import Bag2CSVConfig

class Bag2CSVConverter:
    def __init__(self):
        self.config = Bag2CSVConfig()
        self.config.load_from_ros_params()
        self._init_output_dir()

    def _init_output_dir(self):
        """初始化输出目录"""
        if not exists(self.config.output_csv_dir):
            os.makedirs(self.config.output_csv_dir)
            rospy.loginfo(f"创建输出目录: {self.config.output_csv_dir}")

    def _get_time_value(self, stamp):
        """转换时间戳格式"""
        if self.config.time_format == "s":
            return stamp.to_sec()
        return stamp.to_nsec()

    def _extract_message_fields(self, msg):
        """提取消息字段名和值（递归处理嵌套消息）"""
        fields = []
        values = []
        
        def _recursive_extract(obj, prefix=""):
            if hasattr(obj, "__slots__"):
                for slot in obj.__slots__:
                    if slot.startswith("_"):
                        continue
                    val = getattr(obj, slot)
                    _recursive_extract(val, f"{prefix}{slot}.")
            elif isinstance(obj, list) or isinstance(obj, tuple):
                for i, item in enumerate(obj):
                    _recursive_extract(item, f"{prefix}[{i}].")
            else:
                fields.append(prefix[:-1])  # 移除末尾的"."
                values.append(str(obj))
        
        _recursive_extract(msg)
        return fields, values

    def convert(self):
        """执行bag转csv转换"""
        try:
            with rosbag.Bag(self.config.input_bag_path, "r") as bag:
                # 获取需要处理的话题
                all_topics = bag.get_type_and_topic_info()[1].keys()
                target_topics = self.config.target_topics if self.config.target_topics else all_topics
                
                # 为每个话题创建CSV文件
                csv_writers = {}
                csv_files = {}
                topic_fields = {}  # 缓存每个话题的字段名
                
                for topic, msg, t in bag.read_messages(topics=target_topics):
                    # 初始化CSV文件
                    if topic not in csv_writers:
                        csv_path = join(
                            self.config.output_csv_dir,
                            f"{basename(self.config.input_bag_path).split('.')[0]}_{topic.replace('/', '_')}.csv"
                        )
                        csv_file = open(csv_path, "w", newline="")
                        csv_files[topic] = csv_file
                        csv_writers[topic] = csv.writer(csv_file)
                        rospy.loginfo(f"创建CSV文件: {csv_path}")

                    # 提取消息字段和值
                    if topic not in topic_fields:
                        # 首次处理该话题，提取字段名
                        msg_fields, _ = self._extract_message_fields(msg)
                        topic_fields[topic] = ["timestamp"] + msg_fields
                        if self.config.include_header:
                            csv_writers[topic].writerow(topic_fields[topic])
                    

                    # 写入数据行
                    _, msg_values = self._extract_message_fields(msg)
                    csv_writers[topic].writerow([self._get_time_value(t)] + msg_values)

                # 关闭所有文件
                for f in csv_files.values():
                    f.close()
                rospy.loginfo("转换完成!")

        except Exception as e:
            rospy.logerr(f"转换失败: {str(e)}")
            raise

if __name__ == "__main__":
    try:
        rospy.init_node("bag2csv_node")
        converter = Bag2CSVConverter()
        converter.convert()
    except rospy.ROSInterruptException:
        pass