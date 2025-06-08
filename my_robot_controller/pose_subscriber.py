#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class DiamondPathNavigation(Node):
    def __init__(self):
        super().__init__('pose_subscriber')

        self.pose_subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info('Drawing started...')

    def pose_callback(self, msg = Pose):
        self.get_logger().info(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DiamondPathNavigation()
    rclpy.spin(node)
    rclpy.shutdown()