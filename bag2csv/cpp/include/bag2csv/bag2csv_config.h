#ifndef BAG2CSV_CONFIG_H
#define BAG2CSV_CONFIG_H

#include <string>
#include <vector>
#include <ros/ros.h>

namespace bag2csv {

struct Bag2CSVConfig {
    std::string input_bag_path;      // 输入bag文件路径
    std::string output_csv_dir;      // 输出CSV文件夹
    std::vector<std::string> target_topics;  // 目标话题列表
    bool include_header;             // 是否包含表头
    std::string time_format;         // 时间格式: "ns" 或 "s"

    Bag2CSVConfig() 
        : input_bag_path(""),
          output_csv_dir("./output"),
          include_header(true),
          time_format("ns") {}

    void loadFromRosParams() {
        ros::NodeHandle pnh("~");
        pnh.getParam("input_bag", input_bag_path);
        pnh.getParam("output_dir", output_csv_dir);
        pnh.getParam("target_topics", target_topics);
        pnh.getParam("include_header", include_header);
        pnh.getParam("time_format", time_format);

        // 校验必要参数
        if (input_bag_path.empty()) {
            throw std::invalid_argument("必须设置参数: ~input_bag");
        }
        if (target_topics.empty()) {
            ROS_WARN("未指定目标话题，将转换所有话题");
        }
    }
};

} // namespace bag2csv

#endif // BAG2CSV_CONFIG_H