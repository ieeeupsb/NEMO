#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry

import math
import serial
import subprocess
import struct
import time
import argparse

# list of coordinates for the aruco markers
TILE_MAP = [
    (1.0, 0.5),(2.5,0.5),(4.0,0.5),(5.5,0.5),(8.0,0.5),(10.3,0.5),(12.7,0.5),(15.0,0.5),
    (1.0,2.5),(8.0,2.5),(10.3,2.5),(12.7,2.5),(15.0,2.5),
    (1.0,4.0),(3.3,4.0),(5.7,4.0),(8.0,4.0),(10.3,4.0),(12.7,4.0),(15.0,4.0),
    (1.0,5.5),(3.3,5.5),(5.7,5.5),(8.0,5.5),(15,5.5),
    (1.0,7.5),(3.3,7.5),(5.7,7.5),(8.0,7.5),(10.5,7.5),(12.0,7.5),(13.5,7.5),(15.0,7.5)
]
# start position of robot
ROBOT_START = (5.0,4.0)


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.pos = Point(x=ROBOT_START[0], y=ROBOT_START[1])
        self.target_pos = None
        self.v, self.w = 0.0, 0.0

        # for setting velocity / angular velocity
        self.subscription = self.create_subscription(
            Point,
            'nemo_change_velocity',
            self.velocity_callback,
            1)
        
        # for moving to a tile velocity / angular velocity
        self.subscription = self.create_subscription(
            Int32,
            'nemo_move_to_tile',
            self.tile_callback,
            1)
        
        # odometry subscription to gazebo's odometry data
        # TODO: figure out why we're not receiving odometry data
        #           I have tried subscribing the /odom topic, didn't work
        #           changed the name in the diffdrive model.sdf file to nemo_odometry:
        #                   <odom_topic>nemo_odometry</odom_topic>
        #           no luck there
        #           `ros2 topic list` always shows the topic there
        #           and `ros2 topic echo <topic name>` gives no changes
        #           meaning odometry data is not updated, regardless of the robot moving or not
        self.subscription_odom = self.create_subscription(
            Odometry,
            'nemo_odometry',
            self.odometry_callback,
            1)
        
        # publisher to publish to gazebo
        self.publisher = self.create_publisher(Twist, '/model/diffdrive/cmd_vel', 1)
        
        # bridge parameters on a subprocess
        bridge_cmd = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                      '/model/diffdrive/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist']
        subprocess.Popen(bridge_cmd)

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = self.w
        self.get_logger().info('Publishing linear velocity = {}, angular velocity = {}'.format(self.v, self.w))
        self.publisher.publish(msg)

    def velocity_callback(self, msg):
        self.get_logger().info('Received message: x={}, y={}'.format(msg.x, msg.y))
        self.v = msg.x
        self.w = msg.y
        self.publish_velocity()

    def tile_callback(self, msg):
        tile = TILE_MAP[msg.data]
        self.get_logger().info('Moving to tile: {} - x={}, y={}'.format(msg.data, tile[0], tile[1]))
        self.target_pos = Point(x=tile[0], y=tile[1])
        delta_x = self.target_pos.x - self.pos.x
        delta_y = self.target_pos.y - self.pos.y

        # Calculate the angle and distance to target
        target_angle = math.atan2(delta_y, delta_x)
        distance = math.sqrt(delta_x**2 + delta_y**2)

        # Update linear and angular velocities
        self.v = min(distance, 1.0)  # Limit linear velocity for smooth movement
        self.w = target_angle

        # Publish Twist message to move the robot
        self.publish_velocity()

        # TODO: update local position and implement stopping condition

    # TODO: figure out why we're not receiving odometry
    def odometry_callback(self, msg):
        self.get_logger().info('Odometry data: ({},{})'.format(msg.pose.pose.position.x, msg.pose.pose.position.y))
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y


if __name__ == '__main__':
    rclpy.init()
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()