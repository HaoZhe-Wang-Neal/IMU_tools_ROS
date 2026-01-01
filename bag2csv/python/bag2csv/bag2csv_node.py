from __future__ import division, print_function, unicode_literals

import rospy
import rosbag
import sys
import csv
import os
from sensor_msgs.msg import Imu, MagneticField
from collections import defaultdict

def load_params():
    """加载ROS参数（带默认值）"""
    params = {
        "bag_path": rospy.get_param("~bag_path", "$(find bag2csv)/data/imu.bag"),
        "csv_path": rospy.get_param("~csv_path", "$(find bag2csv)/output/imu_data.csv"),
        "imu_topic": rospy.get_param("~imu_topic", "/imu/data_raw"),
        "mag_topic": rospy.get_param("~mag_topic", "/imu/mag"),
        "export_mag": rospy.get_param("~export_mag", True)
    }
    # 解析ROS路径（替换$(find bag2csv)为实际路径）
    pkg_path = rospy.get_package_path("bag2csv")
    params["bag_path"] = params["bag_path"].replace("$(find bag2csv)", pkg_path)
    params["csv_path"] = params["csv_path"].replace("$(find bag2csv)", pkg_path)
    return params

def main():
    rospy.init_node("bag2csv_node", anonymous=True)
    params = load_params()

    csv_dir = os.path.dirname(params["csv_path"])
    if not os.path.exists(csv_dir):
        os.makedirs(csv_dir)
        rospy.loginfo("Created output directory: %s", csv_dir)

    try:
        with rosbag.Bag(params["bag_path"], "r") as bag:
            # 读取磁力计数据（按时间戳存储）
            mag_data = defaultdict(dict)
            if params["export_mag"]:
                rospy.loginfo("Reading mag data from topic: %s", params["mag_topic"])
                for topic, msg, t in bag.read_messages(topics=[params["mag_topic"]]):
                    if isinstance(msg, MagneticField):
                        timestamp = t.to_sec()
                        mag_data[timestamp] = {
                            "x": msg.magnetic_field.x,
                            "y": msg.magnetic_field.y,
                            "z": msg.magnetic_field.z
                        }

            with open(params["csv_path"], "w", encoding="utf-8", newline="") as f:
                header = ["timestamp", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"]
                if params["export_mag"]:
                    header += ["mag_x", "mag_y", "mag_z"]
                writer = csv.writer(f)
                writer.writerow(header)

                valid_count = 0
                invalid_count = 0
                rospy.loginfo("Reading IMU data from topic: %s", params["imu_topic"])
                for topic, msg, t in bag.read_messages(topics=[params["imu_topic"]]):
                    if not isinstance(msg, Imu):
                        invalid_count += 1
                        continue

                    timestamp = t.to_sec()
                    if timestamp <= 0:
                        rospy.logwarn("Skip invalid IMU data (timestamp=0)")
                        invalid_count += 1
                        continue
                    if msg.linear_acceleration_covariance[0] <= 0 or msg.angular_velocity_covariance[0] <= 0:
                        rospy.logwarn("Skip invalid IMU data (covariance=0, timestamp: %.6f)", timestamp)
                        invalid_count += 1
                        continue

                    # 组装IMU数据行
                    row = [
                        timestamp,
                        msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z,
                        msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z
                    ]

                    # 匹配磁力计数据
                    if params["export_mag"]:
                        mag_row = [0.0, 0.0, 0.0]
                        # 找最接近的时间戳（误差±0.01s）
                        closest_ts = min(mag_data.keys(), key=lambda k: abs(k - timestamp), default=None)
                        if closest_ts and abs(closest_ts - timestamp) < 0.01:
                            mag_row = [
                                mag_data[closest_ts]["x"],
                                mag_data[closest_ts]["y"],
                                mag_data[closest_ts]["z"]
                            ]
                        else:
                            rospy.logwarn("No mag data matched for IMU timestamp: %.6f", timestamp)
                        row += mag_row

                    writer.writerow(row)
                    valid_count += 1

        # 日志统计
        rospy.loginfo("="*50)
        rospy.loginfo("Bag2CSV finished successfully!")
        rospy.loginfo("Valid IMU data written: %d", valid_count)
        rospy.loginfo("Invalid IMU data skipped: %d", invalid_count)
        rospy.loginfo("CSV file saved to: %s", params["csv_path"])
        rospy.loginfo("="*50)

    except Exception as e:
        rospy.logerr("Failed to process bag file: %s", str(e))
        sys.exit(1)

if __name__ == "__main__":
    main()