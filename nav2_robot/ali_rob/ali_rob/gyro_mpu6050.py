#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import mpu6050
import tf2_ros

def main():
    rclpy.init()
    imu_publisher()
    rclpy.shutdown()

def imu_publisher():
    theta = 0
    gyro_x_offset = 0.0
    gyro_y_offset = 0.0
    gyro_z_offset = 0.0
    pub_freq = 10
    count = 0
    num_calibration_iters = 2000
    debug = False

    node = rclpy.create_node('imu_publisher')

    linearaccel_pub = node.create_publisher(Vector3, 'linearaccel', 10)
    gyro_pub = node.create_publisher(Vector3, 'gyro', 10)
    imu_pub = node.create_publisher(Imu, 'imu', 10)

    if node.has_parameter('num_calibration_iters'):
        num_calibration_iters = node.get_parameter('num_calibration_iters').value
    if node.has_parameter('debug'):
        debug = node.get_parameter('debug').value

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    while rclpy.ok():
        mpu = mpu6050.mpu6050(0x68)  # MPU-6050 adresini ve gerekiyorsa diğer ayarları ayarlayın
        gyro_data = mpu.get_gyro_data()

        gyro_x = gyro_data['x']
        gyro_y = gyro_data['y']
        gyro_z = gyro_data['z']

        if count < num_calibration_iters:
            gyro_x_offset += gyro_x
            gyro_y_offset += gyro_y
            gyro_z_offset += gyro_z
            count += 1
        elif count == num_calibration_iters and num_calibration_iters != 0:
            gyro_x_offset /= num_calibration_iters
            gyro_y_offset /= num_calibration_iters
            gyro_z_offset /= num_calibration_iters
            node.get_logger().info("finished calibrating yaw")
            count += 1
        else:
            gyro_x -= gyro_x_offset
            gyro_y -= gyro_y_offset
            gyro_z -= gyro_z_offset

            gyro_msg = Vector3()
            gyro_msg.x = gyro_x
            gyro_msg.y = gyro_y
            gyro_msg.z = gyro_z
            gyro_pub.publish(gyro_msg)

            dt = 1 / pub_freq
            theta += dt * gyro_z

            imu_msg = Imu()
            imu_msg.header.stamp = node.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'base_link'
            q = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]
            imu_msg.orientation_covariance = [0.0] * 9
            imu_msg.angular_velocity_covariance[0] = -1
            imu_msg.linear_acceleration_covariance[0] = -1
            imu_pub.publish(imu_msg)

        rclpy.spin_once(node)

    node.destroy_node()

if __name__ == '__main__':
    main()

