#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from sensor_msgs.msg import Image

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.listener_callback,
            1)
        self.subscription

    def listener_callback(self, msg):
        image = np.zeros((msg.height,msg.width,3),np.uint8)
        image.data = msg.data
        cv2.imshow('Frame', image)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            return

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()