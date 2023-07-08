#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import serial
import struct

def main():
    # 初始化ROS节点和IMU消息发布者
    rospy.init_node('imu_publisher')
    imu_pub = rospy.Publisher('imu_data', Imu, queue_size=10)

    # 配置串口参数
    port = '/dev/ttyUSB0'  # 根据实际情况修改你的串口号
    baudrate = 115200
    ser = serial.Serial(port, baudrate)

    # 设置IMU消息结构体
    imu_msg = Imu()

    # 设置协方差矩阵为-1
    imu_msg.orientation_covariance[0] = -1
    imu_msg.angular_velocity_covariance[0] = -1
    imu_msg.linear_acceleration_covariance[0] = -1

    while not rospy.is_shutdown():
        # 从串口读取一个字节
        byte = ser.read(1)


        if byte == b'\xD4':
            buffer = b''
            while len(buffer) < 44:
                byte = ser.read(1)
                if byte == b'\xD4':  # 如果遇到新的帧头，重置buffer
                    buffer = b''
                else:
                    buffer += byte

            # 添加四个0字节到数据末尾
            data = b'\xD4' + buffer + b'\x00\x00\x00'
            # RVIZ
            imu_msg.header.frame_id = "imu_frame"
            # 解析数据包中的陀螺仪、加速度计和四元数数据
            float_values = struct.unpack('<10f', data[1:1 + 10 * 4])
            imu_msg.header.stamp = rospy.Time.now()  # 使用当前时间作为时间戳
            imu_msg.angular_velocity.x = float_values[0]
            imu_msg.angular_velocity.y = float_values[1]
            imu_msg.angular_velocity.z = float_values[2]

            imu_msg.linear_acceleration.x = float_values[3]
            imu_msg.linear_acceleration.y = float_values[4]
            imu_msg.linear_acceleration.z = float_values[5]

            # 四元数数据存储顺序为w, x, y, z
            imu_msg.orientation.w = float_values[6]
            imu_msg.orientation.x = float_values[7]
            imu_msg.orientation.y = float_values[8]
            imu_msg.orientation.z = float_values[9]

            # 打印角速度和加速度信息
            rospy.loginfo("角速度: x=%f, y=%f, z=%f",
                          imu_msg.angular_velocity.x,
                          imu_msg.angular_velocity.y,
                          imu_msg.angular_velocity.z)
            rospy.loginfo("加速度: x=%f, y=%f, z=%f",
                          imu_msg.linear_acceleration.x,
                          imu_msg.linear_acceleration.y,
                          imu_msg.linear_acceleration.z)
            rospy.loginfo("四元数:w=%f, x=%f, y=%f, z=%f",
                          imu_msg.orientation.w,
                          imu_msg.orientation.x,
                          imu_msg.orientation.y,
                          imu_msg.orientation.z)

            # 发布IMU消息
            imu_pub.publish(imu_msg)

    ser.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
