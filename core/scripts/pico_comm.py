#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist

import serial
import subprocess
import struct
import time
import argparse


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        # self.ser = serial.Serial("/dev/ttyACM0", 115200)
        self.v, self.w = 0.0, 0.0
        # self.encoder_x = 0
        # self.encoder_y = 0
        # self.encoder_theta = 0
        self.subscription = self.create_subscription(
            Point,
            'nemo_subscriber',
            self.callback,
            1)
        
        # publisher to publish to gazebo
        self.publisher = self.create_publisher(Twist, '/model/diffdrive/cmd_vel', 1)
        
        # Attempt to dynamically set the parameter by running the bridge command
        bridge_cmd = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                      '/model/diffdrive/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist']
        subprocess.Popen(bridge_cmd)

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = self.w
        self.get_logger().info('Publishing linear velocity = {}, angular velocity = {}'.format(self.v, self.w))
        self.publisher.publish(msg)


    def callback(self, msg):
        self.get_logger().info('Received message: x={}, y={}'.format(msg.x, msg.y))
        self.v = msg.x
        self.w = msg.y
        self.publish_velocity()
        # self.ser.write(b'\x03' + struct.pack('f',self.v))
        # self.ser.write(b'\x04' + struct.pack('f',self.w))

        # ser.write(b'\x05' + struct.pack('f',self.em))

        # if ser.in_waiting >= 5:
        #     data = ser.read(5)
        #     header = int(data[0])
        #     param = data[1:]
        #     param = struct.unpack('<f', param)[0]
            
        #     print("serial - Received: " + str(messages[header]) + " Param: " + str(param))

        #     # if header == messages.index('DX'):
        #     if header == 13:
        #         encoder_x = param
        #         send_x_y_theta(x_y_t_publisher, encoder_x, encoder_y, encoder_theta)
                
        #     elif header == 14:
        #         encoder_y = param
        #         send_x_y_theta(x_y_t_publisher, encoder_x, encoder_y, encoder_theta)
                
        #     elif header == 15:
        #         encoder_theta = param
        #         send_x_y_theta(x_y_t_publisher, encoder_x, encoder_y, encoder_theta)

if __name__ == '__main__':
    rclpy.init()
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()