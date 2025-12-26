"""Bag2CSV 配置参数类"""
class Bag2CSVConfig:
    def __init__(self):
        # 默认配置
        self.input_bag_path = ""          # 输入bag文件路径
        self.output_csv_dir = "./output"  # 输出CSV文件夹
        self.target_topics = []           # 需要转换的话题列表
        self.include_header = True        # 是否包含CSV表头
        self.time_format = "ns"           # 时间格式: "ns" 

    def load_from_ros_params(self):
        """从ROS参数服务器加载配置"""
        import rospy
        self.input_bag_path = rospy.get_param("~input_bag", self.input_bag_path)
        self.output_csv_dir = rospy.get_param("~output_dir", self.output_csv_dir)
        self.target_topics = rospy.get_param("~target_topics", self.target_topics)
        self.include_header = rospy.get_param("~include_header", self.include_header)
        self.time_format = rospy.get_param("~time_format", self.time_format)

        # 校验必要参数
        if not self.input_bag_path:
            raise ValueError("必须设置输入bag文件路径: ~input_bag")
        if not self.target_topics:
            rospy.logwarn("未指定目标话题，将转换所有话题")