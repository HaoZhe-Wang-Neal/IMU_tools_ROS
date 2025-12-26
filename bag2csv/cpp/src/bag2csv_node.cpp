#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <typeinfo>
#include "bag2csv/bag2csv_config.h"

namespace bag2csv {

class Bag2CSVConverter {
private:
    Bag2CSVConfig config_;
    std::map<std::string, std::ofstream> csv_files_;
    std::map<std::string, std::vector<std::string>> topic_fields_;

public:
    Bag2CSVConverter() {
        config_.loadFromRosParams();
        initOutputDir();
    }

    void initOutputDir() {
        boost::filesystem::path dir(config_.output_csv_dir);
        if (!boost::filesystem::exists(dir)) {
            boost::filesystem::create_directories(dir);
            ROS_INFO_STREAM("创建输出目录: " << config_.output_csv_dir);
        }
    }

    double getTimeValue(const ros::Time& stamp) {
        if (config_.time_format == "s") {
            return stamp.toSec();
        } else {
            return stamp.toNSec();
        }
    }

    // 递归提取消息字段名和值 以字符串形式返回
    void extractMessageFields(const ros::Message& msg, std::vector<std::string>& fields, 
                             std::vector<std::string>& values, std::string prefix = "") {
        const std::type_info& type = typeid(msg);
        std::string type_name = type.name();

        // 处理常见IMU相关消息类型（可根据需要扩展）
        if (type_name.find("sensor_msgs::Imu") != std::string::npos) {
            auto& imu = dynamic_cast<const sensor_msgs::Imu&>(msg);
            // 线性加速度
            extractVector3(imu.linear_acceleration, "linear_acceleration", fields, values, prefix);
            // 角速度
            extractVector3(imu.angular_velocity, "angular_velocity", fields, values, prefix);
            // 姿态四元数
            extractQuaternion(imu.orientation, "orientation", fields, values, prefix);
        }

        else if (type_name.find("sensor_msgs::MagneticField") != std::string::npos) {
            auto& mag = dynamic_cast<const sensor_msgs::MagneticField&>(msg);
            extractVector3(mag.magnetic_field, "magnetic_field", fields, values, prefix);
        }
        
        
        else {
            ROS_WARN_STREAM("不支持的消息类型: " << type_name << "，将跳过字段提取");
        }
    }


    // 提取Vector3字段
    void extractVector3(const geometry_msgs::Vector3& vec, const std::string& name,
                       std::vector<std::string>& fields, std::vector<std::string>& values,
                       const std::string& prefix) {
        fields.push_back(prefix + name + ".x");
        fields.push_back(prefix + name + ".y");
        fields.push_back(prefix + name + ".z");
        values.push_back(std::to_string(vec.x));
        values.push_back(std::to_string(vec.y));
        values.push_back(std::to_string(vec.z));
    }

    // 提取Quaternion字段
    void extractQuaternion(const geometry_msgs::Quaternion& quat, const std::string& name,
                          std::vector<std::string>& fields, std::vector<std::string>& values,
                          const std::string& prefix) {
        fields.push_back(prefix + name + ".x");
        fields.push_back(prefix + name + ".y");
        fields.push_back(prefix + name + ".z");
        fields.push_back(prefix + name + ".w");
        values.push_back(std::to_string(quat.x));
        values.push_back(std::to_string(quat.y));
        values.push_back(std::to_string(quat.z));
        values.push_back(std::to_string(quat.w));
    }


    // 生成CSV文件名
    std::string generateCsvPath(const std::string& topic) {
        std::string topic_clean = topic;
        std::replace(topic_clean.begin(), topic_clean.end(), '/', '_');
        boost::filesystem::path bag_path(config_.input_bag_path);
        std::string bag_name = bag_path.stem().string();
        return config_.output_csv_dir + "/" + bag_name + "_" + topic_clean + ".csv";
    }

    
    void convert() {
        try {
            rosbag::Bag bag;
            bag.open(config_.input_bag_path, rosbag::bagmode::Read);

            // 获取所有话题
            std::vector<std::string> all_topics;
            for (const auto& topic_info : bag.getTopicTypes()) {
                all_topics.push_back(topic_info.first);
            }

            // 确定目标话题
            std::vector<std::string> target_topics = config_.target_topics;
            if (target_topics.empty()) {
                target_topics = all_topics;
            }

            // 遍历bag消息
            rosbag::View view(bag, rosbag::TopicQuery(target_topics));
            for (const rosbag::MessageInstance& msg : view) {
                std::string topic = msg.getTopic();
                ros::Time stamp = msg.getTime();

                // 初始化CSV文件
                if (csv_files_.find(topic) == csv_files_.end()) {
                    std::string csv_path = generateCsvPath(topic);
                    csv_files_[topic].open(csv_path);
                    if (!csv_files_[topic].is_open()) {
                        throw std::runtime_error("无法打开CSV文件: " + csv_path);
                    }
                    ROS_INFO_STREAM("创建CSV文件: " << csv_path);

                    // 提取字段名并写入表头
                    std::vector<std::string> fields, values;
                    extractMessageFields(*msg.getMessage(), fields, values);
                    topic_fields_[topic] = fields;

                    if (config_.include_header) {
                        csv_files_[topic] << "timestamp,";
                        for (size_t i = 0; i < fields.size(); ++i) {
                            csv_files_[topic] << fields[i];
                            if (i != fields.size() - 1) csv_files_[topic] << ",";
                        }
                        csv_files_[topic] << "\n";
                    }
                }

                // 写入数据行
                std::vector<std::string> values;
                extractMessageFields(*msg.getMessage(), topic_fields_[topic], values);
                
                csv_files_[topic] << getTimeValue(stamp) << ",";
                for (size_t i = 0; i < values.size(); ++i) {
                    csv_files_[topic] << values[i];
                    if (i != values.size() - 1) csv_files_[topic] << ",";
                }
                csv_files_[topic] << "\n";
            }

            // 关闭所有文件
            for (auto& pair : csv_files_) {
                pair.second.close();
            }
            bag.close();
            ROS_INFO("转换完成!");
        }
        catch (const std::exception& e) {
            ROS_ERROR_STREAM("转换失败: " << e.what());
            throw;
        }
    }
};

} // namespace bag2csv

int main(int argc, char**argv) {
    ros::init(argc, argv, "bag2csv_node");
    try {
        bag2csv::Bag2CSVConverter converter;
        converter.convert();
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("节点启动失败: " << e.what());
        return 1;
    }
    return 0;
}