#!/usr/bin/env python
import time

import rospy
from sensor_msgs.msg import Imu, NavSatFix
import serial
import struct
import tf2_ros
from geometry_msgs.msg import TransformStamped
from numpy import linalg
import numpy as np

class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data = []

    def add_value(self, value):
        self.data.append(value)
        if len(self.data) > self.window_size:
            self.data.pop(0)

    def get_average(self):
        return sum(self.data) / len(self.data) if self.data else 0


class MovingMedianFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data = []

    def add_value(self, value):
        self.data.append(value)
        if len(self.data) > self.window_size:
            self.data.pop(0)

    def get_median(self):
        return np.median(self.data) if self.data else 0
class IMUPublisher:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        rospy.init_node('imu_publisher')
        self.imu_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)
        self.navsatfix_pub = rospy.Publisher('navsatfix', NavSatFix, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.window_size = 10
        self.w_filter = MovingMedianFilter(self.window_size)
        self.x_filter = MovingMedianFilter(self.window_size)
        self.y_filter = MovingMedianFilter(self.window_size)
        self.z_filter = MovingMedianFilter(self.window_size)

    def normalize_quaternion(self, q):
        norm = linalg.norm([q.w, q.x, q.y, q.z])
        if norm > 1e-6:
            q.w /= norm
            q.x /= norm
            q.y /= norm
            q.z /= norm
        else:
            rospy.logwarn("Quaternion norm is close to zero, using default quaternion values.")
            q.w = 1.0
            q.x = 0.0
            q.y = 0.0
            q.z = 0.0
        return q

    def publish_imu_transform(self, imu_orientation):
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "imu_frame"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.w = imu_orientation.w
        t.transform.rotation.x = imu_orientation.x
        t.transform.rotation.y = imu_orientation.y
        t.transform.rotation.z = imu_orientation.z

        self.tf_broadcaster.sendTransform(t)

    def process_imu_data(self, ser, imu_msg):
        byte = ser.read(1)

        if byte == b'\xD4':
            buffer = b''
            while len(buffer) < 40:
                byte = ser.read(1)
                if byte == b'\xD4':
                    buffer = b''
                else:
                    buffer += byte

            data = b'\xD4' + buffer
            imu_msg.header.frame_id = "imu_frame"
            float_values = struct.unpack('<10f', data[1:1 + 10 * 4])
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.angular_velocity.x = float_values[0]
            imu_msg.angular_velocity.y = float_values[1]
            imu_msg.angular_velocity.z = float_values[2]

            imu_msg.linear_acceleration.x = float_values[3]
            imu_msg.linear_acceleration.y = float_values[4]
            imu_msg.linear_acceleration.z = float_values[5]

            quat_norm = linalg.norm([float_values[6], float_values[7], float_values[8], float_values[9]])

            if quat_norm > 1e-6:
                w_filtered = self.w_filter.add_value(float_values[6] / quat_norm)
                x_filtered = self.x_filter.add_value(float_values[7] / quat_norm)
                y_filtered = self.y_filter.add_value(float_values[8] / quat_norm)
                z_filtered = self.z_filter.add_value(float_values[9] / quat_norm)

                imu_msg.orientation.w = self.w_filter.get_median()
                imu_msg.orientation.x = self.x_filter.get_median()
                imu_msg.orientation.y = self.y_filter.get_median()
                imu_msg.orientation.z = self.z_filter.get_median()
            else:
                rospy.logwarn("Quaternion norm is close to zero, using default quaternion values")
                imu_msg.orientation.w = 1.0
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = 0.0

            rospy.loginfo("Angular velocity: x=%f, y=%f, z=%f",
                          imu_msg.angular_velocity.x,
                          imu_msg.angular_velocity.y,
                          imu_msg.angular_velocity.z)
            rospy.loginfo("Linear acceleration: x=%f, y=%f, z=%f",
                          imu_msg.linear_acceleration.x,
                          imu_msg.linear_acceleration.y,
                          imu_msg.linear_acceleration.z)
            rospy.loginfo("Quaternion: w=%f, x=%f, y=%f, z=%f",
                          imu_msg.orientation.w,
                          imu_msg.orientation.x,
                          imu_msg.orientation.y,
                          imu_msg.orientation.z)

            # Publish IMU message
            self.imu_pub.publish(imu_msg)
            # Normalize quaternion before publishing the transform
            normalized_quaternion = self.normalize_quaternion(imu_msg.orientation)
            self.publish_imu_transform(normalized_quaternion)

    def run(self):
        ser = serial.Serial(self.port, self.baudrate, timeout=1)
        imu_msg = Imu()
        while not rospy.is_shutdown():
            try:
                self.process_imu_data(ser, imu_msg)
            except serial.serialutil.SerialException as e:
                rospy.logerr("Error reading from IMU: %s", str(e))
                rospy.loginfo("Attempting to reconnect...")
                ser.close()
                time.sleep(1)
                try:
                    ser.open()
                except serial.serialutil.SerialException as e:
                    rospy.logerr("Failed to reconnect: %s", str(e))

if __name__ == '__main__':
    try:
        imu_publisher = IMUPublisher('/dev/ttyUSB0', 115200)  # Replace with your desired port and baudrate
        imu_publisher.run()
    except rospy.ROSInterruptException:
        pass