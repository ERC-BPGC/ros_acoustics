#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

class OdomSubscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')
        self.sub = self.create_subscription(Odometry, 'sprintbot/odom', self.odom_callback, 1)
        self.i = 0

    def odom_callback(self, msg):
        print('Received odometry message: ' + str(self.i))
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    rclpy.spin(odom_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()