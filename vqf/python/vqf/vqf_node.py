#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3
import threading

class VQF:
    """VQF姿态解算核心类"""
    def __init__(self, is_9axis, kp_acc, ki_acc, kp_mag, ki_mag,
                 gyro_noise, accel_noise, mag_noise, sample_freq):
        self.is_9axis = is_9axis
        self.kp_acc = kp_acc
        self.ki_acc = ki_acc
        self.kp_mag = kp_mag
        self.ki_mag = ki_mag
        self.gyro_noise = gyro_noise
        self.accel_noise = accel_noise
        self.mag_noise = mag_noise
        self.sample_freq = sample_freq
        self.dt = 1.0 / sample_freq

        # 初始化姿态四元数 [w, x, y, z]
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        # 积分误差
        self.integral_error_acc = np.zeros(3)
        self.integral_error_mag = np.zeros(3)
        # 线程锁
        self.lock = threading.Lock()

    def normalize_quat(self, q):
        """归一化四元数"""
        norm = np.linalg.norm(q)
        if norm < 1e-6:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return q / norm

    def quat_mult(self, q1, q2):
        """四元数乘法: q1 * q2"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        return self.normalize_quat([w, x, y, z])

    def quat_to_rot(self, q):
        """四元数转旋转矩阵"""
        w, x, y, z = q
        return np.array([
            [1-2*y²-2*z², 2*x*y-2*z*w, 2*x*z+2*y*w],
            [2*x*y+2*z*w, 1-2*x²-2*z², 2*y*z-2*x*w],
            [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x²-2*y²]
        ])

    def update(self, gyro, accel, mag=np.zeros(3)):
        """更新姿态"""
        with self.lock:
            # 1. 预测步（陀螺仪积分）
            gyro_omega = gyro * self.dt
            theta = np.linalg.norm(gyro_omega)
            if theta > 1e-6:
                q_gyro = np.array([
                    np.cos(theta/2),
                    np.sin(theta/2)*gyro_omega[0]/theta,
                    np.sin(theta/2)*gyro_omega[1]/theta,
                    np.sin(theta/2)*gyro_omega[2]/theta
                ])
            else:
                q_gyro = np.array([1.0, 0.5*gyro_omega[0], 0.5*gyro_omega[1], 0.5*gyro_omega[2]])
            self.q = self.quat_mult(self.q, q_gyro)

            # 2. 加速度计校正
            if np.linalg.norm(accel) > 0.1:
                accel_norm = accel / np.linalg.norm(accel)
                # 期望重力向量 [0,0,-1] 旋转到机体坐标系
                rot = self.quat_to_rot(self.q)
                gravity = rot @ np.array([0, 0, 1])
                # 计算误差
                error_acc = np.cross(accel_norm, gravity)
                self.integral_error_acc += error_acc * self.dt
                # 校正量
                correction_acc = self.kp_acc * error_acc + self.ki_acc * self.integral_error_acc
                # 应用校正
                theta_corr = np.linalg.norm(correction_acc) * self.dt
                if theta_corr > 1e-6:
                    q_corr = np.array([
                        np.cos(theta_corr/2),
                        np.sin(theta_corr/2)*correction_acc[0]/theta_corr,
                        np.sin(theta_corr/2)*correction_acc[1]/theta_corr,
                        np.sin(theta_corr/2)*correction_acc[2]/theta_corr
                    ])
                else:
                    q_corr = np.array([1.0, 0.5*correction_acc[0]*self.dt, 0.5*correction_acc[1]*self.dt, 0.5*correction_acc[2]*self.dt])
                self.q = self.quat_mult(self.q, q_corr)

            # 3. 磁力计校正（9轴）
            if self.is_9axis and np.linalg.norm(mag) > 0.1:
                mag_norm = mag / np.linalg.norm(mag)
                # 期望地磁向 [1,0,0] 旋转到机体坐标系
                rot = self.quat_to_rot(self.q)
                mag_earth = rot @ np.array([1, 0, 0])
                mag_earth[2] = 0  # 投影到水平面
                mag_earth = mag_earth / np.linalg.norm(mag_earth)
                # 计算误差
                error_mag = np.cross(mag_norm, mag_earth)
                self.integral_error_mag += error_mag * self.dt
                # 校正量
                correction_mag = self.kp_mag * error_mag + self.ki_mag * self.integral_error_mag
                # 应用校正
                theta_corr = np.linalg.norm(correction_mag) * self.dt
                if theta_corr > 1e-6:
                    q_corr = np.array([
                        np.cos(theta_corr/2),
                        np.sin(theta_corr/2)*correction_mag[0]/theta_corr,
                        np.sin(theta_corr/2)*correction_mag[1]/theta_corr,
                        np.sin(theta_corr/2)*correction_mag[2]/theta_corr
                    ])
                else:
                    q_corr = np.array([1.0, 0.5*correction_mag[0]*self.dt, 0.5*correction_mag[1]*self.dt, 0.5*correction_mag[2]*self.dt])
                self.q = self.quat_mult(self.q, q_corr)

    def get_quaternion(self):
        """获取四元数 [x,y,z,w] (ROS格式)"""
        with self.lock:
            w, x, y, z = self.q
            return [x, y, z, w]

    def get_rpy(self):
        """获取RPY（弧度）"""
        with self.lock:
            w, x, y, z = self.q
            # Roll (x-axis)
            roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x² + y²))
            # Pitch (y-axis)
            pitch = np.arcsin(2*(w*y - z*x))
            # Yaw (z-axis)
            yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y² + z²))
            return [roll, pitch, yaw]

class VQFNode:
    """VQF ROS节点"""
    def __init__(self):
        rospy.init_node('vqf_node', anonymous=True)
        self.load_params()

        # 初始化VQF
        self.vqf = VQF(
            is_9axis=self.is_9axis,
            kp_acc=self.kp_acc,
            ki_acc=self.ki_acc,
            kp_mag=self.kp_mag,
            ki_mag=self.ki_mag,
            gyro_noise=self.gyro_noise,
            accel_noise=self.accel_noise,
            mag_noise=self.mag_noise,
            sample_freq=self.sample_freq
        )
        self.vqf.q = np.array(self.init_orientation)

        # 数据缓存
        self.last_gyro = np.zeros(3)
        self.last_accel = np.zeros(3)
        self.last_mag = np.zeros(3)
        self.has_mag_data = False
        self.data_lock = threading.Lock()

        # 订阅/发布
        self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=100)
        if self.is_9axis:
            self.mag_sub = rospy.Subscriber(self.mag_topic, MagneticField, self.mag_callback, queue_size=100)
        self.pose_pub = rospy.Publisher(self.output_topic, Quaternion, queue_size=100)
        self.rpy_pub = rospy.Publisher(self.output_rpy_topic, Vector3, queue_size=100)

        # 定时发布
        self.publish_timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_callback)

    def load_params(self):
        """加载ROS参数"""
        self.imu_topic = rospy.get_param("~vqf/imu_topic", "/imu/cleaned")
        self.mag_topic = rospy.get_param("~vqf/mag_topic", "/imu/mag")
        self.output_topic = rospy.get_param("~vqf/output_topic", "/imu/vqf/pose")
        self.output_rpy_topic = rospy.get_param("~vqf/output_rpy_topic", "/imu/vqf/rpy")
        self.frame_id = rospy.get_param("~vqf/frame_id", "imu_link")
        self.is_9axis = rospy.get_param("~vqf/is_9axis", True)
        self.publish_rate = rospy.get_param("~vqf/publish_rate", 100.0)
        self.kp_acc = rospy.get_param("~vqf/kp_acc", 1.0)
        self.ki_acc = rospy.get_param("~vqf/ki_acc", 0.01)
        self.kp_mag = rospy.get_param("~vqf/kp_mag", 0.5)
        self.ki_mag = rospy.get_param("~vqf/ki_mag", 0.005)
        self.gyro_noise = rospy.get_param("~vqf/gyro_noise", 0.01)
        self.accel_noise = rospy.get_param("~vqf/accel_noise", 0.1)
        self.mag_noise = rospy.get_param("~vqf/mag_noise", 0.05)
        self.sample_freq = rospy.get_param("~vqf/sample_freq", 100.0)
        self.init_orientation = rospy.get_param("~vqf/init_orientation", [0.0, 0.0, 0.0, 1.0])

    def imu_callback(self, msg):
        """IMU数据回调"""
        with self.data_lock:
            self.last_gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            self.last_accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            # 更新VQF
            if self.is_9axis and self.has_mag_data:
                self.vqf.update(self.last_gyro, self.last_accel, self.last_mag)
                self.has_mag_data = False
            else:
                self.vqf.update(self.last_gyro, self.last_accel)

    def mag_callback(self, msg):
        """磁力计数据回调"""
        with self.data_lock:
            self.last_mag = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
            self.has_mag_data = True

    def publish_callback(self, event):
        """发布姿态数据"""
        # 发布四元数
        q = self.vqf.get_quaternion()
        pose_msg = Quaternion()
        pose_msg.x = q[0]
        pose_msg.y = q[1]
        pose_msg.z = q[2]
        pose_msg.w = q[3]
        self.pose_pub.publish(pose_msg)

        # 发布RPY
        rpy = self.vqf.get_rpy()
        rpy_msg = Vector3()
        rpy_msg.x = rpy[0]
        rpy_msg.y = rpy[1]
        rpy_msg.z = rpy[2]
        self.rpy_pub.publish(rpy_msg)

    def run(self):
        rospy.spin()



def main():
    try:
        node = VQFNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
        

if __name__ == "__main__":
    main() 