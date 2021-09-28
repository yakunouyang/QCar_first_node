#!/usr/bin/env python
import rospy
import numpy as np
import tf
from scipy.spatial.transform.rotation import Rotation
from qcar.product_QCar import QCar

from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu

import time


class QcarNode:
    def __init__(self):
        self.battery_pub_ = rospy.Publisher('/qcar/battery_state', BatteryState, queue_size=10)
        self.velocity_pub_ = rospy.Publisher('/qcar/velocity', TwistWithCovarianceStamped, queue_size=10)
        self.imu_pub_ = rospy.Publisher('/qcar/imu', Imu, queue_size=1)
        self.odom_pub_ = rospy.Publisher('/qcar/odom', Odometry, queue_size=1)

        self.velocity = 0.0
        self.steering = 0.0

        self.real_velocity = 0.0
        self.last_count = 0.0
        self.last_time = time.time()
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_last_time = 0.0
        self.br = tf.TransformBroadcaster()

        self.yaw = 0.0
        self.yaw_last_time = 0.0

        self.car = QCar()
        self.sample_time = rospy.Rate(100)
        self.cmd_sub_ = rospy.Subscriber('/cmd_vel', Twist, self.process_cmd, queue_size=100)
        self.imu_timer = rospy.Timer(rospy.Rate(100).sleep_dur, self.read_imu)
        self.std_timer = rospy.Timer(rospy.Rate(50).sleep_dur, self.read_std)

    def read_imu(self, evt):
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base'

        [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
         msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z] = self.car.read_IMU()

        dt = time.time() - self.yaw_last_time if self.yaw_last_time > 0 else 0.0
        self.yaw += msg.angular_velocity.z * dt
        self.yaw_last_time = time.time()

        (msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w) = Rotation.from_euler('xyz', [0, 0, self.yaw]).as_quat()

        msg.angular_velocity_covariance = np.concatenate(np.identity(3)).tolist()
        msg.linear_acceleration_covariance = np.concatenate(np.identity(3)).tolist()

        msg.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1]

        self.imu_pub_.publish(msg)

    def read_std(self, evt):
        [motor_current, voltage, motor_encoder] = self.car.read_std()

        battery_state = BatteryState()
        battery_state.header.stamp = rospy.Time.now()
        battery_state.header.frame_id = 'base'
        battery_state.voltage = voltage
        self.battery_pub_.publish(battery_state)

        car_travel = (1.0 / 720 / 4) * ((13.0 * 19) / (70.0 * 37)) * 1 * 2 * np.pi * 0.0342 * motor_encoder
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base'

        self.real_velocity = (car_travel - self.last_count) / (time.time() - self.last_time)
        self.last_time = time.time()
        self.last_count = car_travel
        msg.twist.twist.linear.x = self.real_velocity
        msg.twist.covariance = [0.1] + [0] * 35  # only x
        self.velocity_pub_.publish(msg)

    def process_cmd(self, cmd):
        self.velocity = cmd.linear.x
        self.steering = cmd.angular.z + 0.05
        command = np.array([self.velocity, self.steering])
        LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.car.write_std(command, LEDs)

    def terminate(self):
        self.car.terminate()


if __name__ == '__main__':
    rospy.init_node('qcar_example_node')
    r = QcarNode()
    rospy.spin()
    r.terminate()
