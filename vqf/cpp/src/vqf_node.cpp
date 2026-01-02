#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Header.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <cmath>
#include <vector>

// VQF算法核心类
class VQF {
public:
    VQF(bool is_9axis, double kp_acc, double ki_acc, double kp_mag, double ki_mag,
        double gyro_noise, double accel_noise, double mag_noise, double sample_freq) 
        : is_9axis_(is_9axis), kp_acc_(kp_acc), ki_acc_(ki_acc), kp_mag_(kp_mag), ki_mag_(ki_mag),
          gyro_noise_(gyro_noise), accel_noise_(accel_noise), mag_noise_(mag_noise), sample_freq_(sample_freq) {
        // 初始化四元数
        q_.w() = 1.0; q_.x() = 0.0; q_.y() = 0.0; q_.z() = 0.0;
        integral_error_acc_.setZero();
        integral_error_mag_.setZero();
        dt_ = 1.0 / sample_freq_;
    }

    // 更新姿态（输入角速度、加速度、磁力计）
    void update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag = Eigen::Vector3d::Zero()) {
        std::lock_guard<std::mutex> lock(mutex_);

        // 1. 预测步（陀螺仪积分）
        Eigen::Quaterniond q_gyro;
        Eigen::Vector3d gyro_omega = gyro * dt_;
        double theta = gyro_omega.norm();
        if (theta > 1e-6) {
            q_gyro = Eigen::Quaterniond(cos(theta/2), 
                                        sin(theta/2)*gyro_omega.x()/theta,
                                        sin(theta/2)*gyro_omega.y()/theta,
                                        sin(theta/2)*gyro_omega.z()/theta);
        } else {
            q_gyro = Eigen::Quaterniond(1.0, 0.5*gyro_omega.x(), 0.5*gyro_omega.y(), 0.5*gyro_omega.z());
        }
        q_ = q_ * q_gyro;
        q_.normalize();

        // 2. 校正步（加速度计）
        if (accel.norm() > 0.1) { // 有效加速度数据
            Eigen::Vector3d accel_norm = accel.normalized();
            Eigen::Vector3d gravity = q_ * Eigen::Vector3d(0, 0, -1); // 期望重力向量
            Eigen::Vector3d error_acc = accel_norm.cross(gravity);
            
            integral_error_acc_ += error_acc * dt_;
            Eigen::Vector3d correction_acc = kp_acc_ * error_acc + ki_acc_ * integral_error_acc_;
            
            // 应用校正
            Eigen::Quaterniond q_corr_acc;
            theta = correction_acc.norm() * dt_;
            if (theta > 1e-6) {
                q_corr_acc = Eigen::Quaterniond(cos(theta/2), 
                                                sin(theta/2)*correction_acc.x()/theta,
                                                sin(theta/2)*correction_acc.y()/theta,
                                                sin(theta/2)*correction_acc.z()/theta);
            } else {
                q_corr_acc = Eigen::Quaterniond(1.0, 0.5*correction_acc.x()*dt_, 0.5*correction_acc.y()*dt_, 0.5*correction_acc.z()*dt_);
            }
            q_ = q_ * q_corr_acc;
            q_.normalize();
        }

        // 3. 校正步（磁力计，9轴启用）
        if (is_9axis_ && mag.norm() > 0.1) {
            Eigen::Vector3d mag_norm = mag.normalized();
            Eigen::Vector3d mag_earth = q_ * Eigen::Vector3d(1, 0, 0); // 期望地磁北向
            mag_earth.z() = 0; // 投影到水平面
            mag_earth.normalize();
            
            Eigen::Vector3d error_mag = mag_norm.cross(mag_earth);
            integral_error_mag_ += error_mag * dt_;
            Eigen::Vector3d correction_mag = kp_mag_ * error_mag + ki_mag_ * integral_error_mag_;
            
            // 应用校正
            Eigen::Quaterniond q_corr_mag;
            theta = correction_mag.norm() * dt_;
            if (theta > 1e-6) {
                q_corr_mag = Eigen::Quaterniond(cos(theta/2), 
                                                sin(theta/2)*correction_mag.x()/theta,
                                                sin(theta/2)*correction_mag.y()/theta,
                                                sin(theta/2)*correction_mag.z()/theta);
            } else {
                q_corr_mag = Eigen::Quaterniond(1.0, 0.5*correction_mag.x()*dt_, 0.5*correction_mag.y()*dt_, 0.5*correction_mag.z()*dt_);
            }
            q_ = q_ * q_corr_mag;
            q_.normalize();
        }
    }

    // 获取当前姿态四元数
    Eigen::Quaterniond getQuaternion() {
        std::lock_guard<std::mutex> lock(mutex_);
        return q_;
    }

    // 获取RPY（弧度）
    Eigen::Vector3d getRPY() {
        std::lock_guard<std::mutex> lock(mutex_);
        return q_.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw
    }

private:
    bool is_9axis_;
    double kp_acc_, ki_acc_, kp_mag_, ki_mag_;
    double gyro_noise_, accel_noise_, mag_noise_, sample_freq_, dt_;
    
    Eigen::Quaterniond q_; // 姿态四元数
    Eigen::Vector3d integral_error_acc_; // 加速度计积分误差
    Eigen::Vector3d integral_error_mag_; // 磁力计积分误差
    std::mutex mutex_;
};

// ROS节点类
class VQFNode {
public:
    VQFNode(ros::NodeHandle& nh) : nh_(nh) {
        // 加载参数
        loadParams();

        // 初始化VQF
        vqf_ = std::make_unique<VQF>(is_9axis_, kp_acc_, ki_acc_, kp_mag_, ki_mag_,
                                     gyro_noise_, accel_noise_, mag_noise_, sample_freq_);

        // 订阅话题
        imu_sub_ = nh_.subscribe(imu_topic_, 100, &VQFNode::imuCallback, this);
        if (is_9axis_) {
            mag_sub_ = nh_.subscribe(mag_topic_, 100, &VQFNode::magCallback, this);
        }

        // 发布话题
        pose_pub_ = nh_.advertise<geometry_msgs::Quaternion>(output_topic_, 100);
        rpy_pub_ = nh_.advertise<geometry_msgs::Vector3>(output_rpy_topic_, 100);

        // 定时发布
        publish_timer_ = nh_.createTimer(ros::Duration(1.0/publish_rate_), &VQFNode::publishCallback, this);
    }

    // IMU回调函数
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // 存储IMU数据
        last_gyro_ = Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
        last_accel_ = Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
        last_imu_stamp_ = imu_msg->header.stamp;

        // 更新VQF
        if (is_9axis_ && has_mag_data_) {
            vqf_->update(last_gyro_, last_accel_, last_mag_);
            has_mag_data_ = false; // 重置磁力计标记
        } else {
            vqf_->update(last_gyro_, last_accel_);
        }
    }

    // 磁力计回调函数
    void magCallback(const sensor_msgs::MagneticField::ConstPtr& mag_msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_mag_ = Eigen::Vector3d(mag_msg->magnetic_field.x, mag_msg->magnetic_field.y, mag_msg->magnetic_field.z);
        has_mag_data_ = true;
    }

    // 发布回调函数
    void publishCallback(const ros::TimerEvent& e) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // 发布四元数
        Eigen::Quaterniond q = vqf_->getQuaternion();
        geometry_msgs::Quaternion pose_msg;
        pose_msg.x = q.x();
        pose_msg.y = q.y();
        pose_msg.z = q.z();
        pose_msg.w = q.w();
        pose_pub_.publish(pose_msg);

        // 发布RPY
        Eigen::Vector3d rpy = vqf_->getRPY();
        geometry_msgs::Vector3 rpy_msg;
        rpy_msg.x = rpy.x();
        rpy_msg.y = rpy.y();
        rpy_msg.z = rpy.z();
        rpy_pub_.publish(rpy_msg);
    }

private:
    void loadParams() {
        nh_.param<std::string>("vqf/imu_topic", imu_topic_, "/imu/cleaned");
        nh_.param<std::string>("vqf/mag_topic", mag_topic_, "/imu/mag");
        nh_.param<std::string>("vqf/output_topic", output_topic_, "/imu/vqf/pose");
        nh_.param<std::string>("vqf/output_rpy_topic", output_rpy_topic_, "/imu/vqf/rpy");
        nh_.param<std::string>("vqf/frame_id", frame_id_, "imu_link");
        nh_.param<bool>("vqf/is_9axis", is_9axis_, true);
        nh_.param<double>("vqf/publish_rate", publish_rate_, 100.0);
        nh_.param<double>("vqf/kp_acc", kp_acc_, 1.0);
        nh_.param<double>("vqf/ki_acc", ki_acc_, 0.01);
        nh_.param<double>("vqf/kp_mag", kp_mag_, 0.5);
        nh_.param<double>("vqf/ki_mag", ki_mag_, 0.005);
        nh_.param<double>("vqf/gyro_noise", gyro_noise_, 0.01);
        nh_.param<double>("vqf/accel_noise", accel_noise_, 0.1);
        nh_.param<double>("vqf/mag_noise", mag_noise_, 0.05);
        nh_.param<double>("vqf/sample_freq", sample_freq_, 100.0);
        
        // 初始姿态
        std::vector<double> init_q = {0.0, 0.0, 0.0, 1.0};
        nh_.param<std::vector<double>>("vqf/init_orientation", init_q, init_q);
        init_q_ = Eigen::Quaterniond(init_q[3], init_q[0], init_q[1], init_q[2]);
    }

    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_, mag_sub_;
    ros::Publisher pose_pub_, rpy_pub_;
    ros::Timer publish_timer_;

    // 参数
    std::string imu_topic_, mag_topic_, output_topic_, output_rpy_topic_, frame_id_;
    bool is_9axis_;
    double publish_rate_, kp_acc_, ki_acc_, kp_mag_, ki_mag_;
    double gyro_noise_, accel_noise_, mag_noise_, sample_freq_;
    Eigen::Quaterniond init_q_;

    // VQF实例
    std::unique_ptr<VQF> vqf_;

    // 数据缓存
    Eigen::Vector3d last_gyro_, last_accel_, last_mag_;
    ros::Time last_imu_stamp_;
    bool has_mag_data_ = false;
    std::mutex data_mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vqf_node");
    ros::NodeHandle nh;
    VQFNode vqf_node(nh);
    ros::spin();
    return 0;
}