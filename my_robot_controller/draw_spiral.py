#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class Spiral(Node):
    def __init__(self):
        super().__init__('draw_spiral')

        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose, '/turtle1/pose', self.move, 10)

        self.get_logger().info('Drawing started...')
        self.start_time = self.get_clock().now()

    def move(self, pose: Pose):
        cmd = Twist()
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # these values are calculated considering the turtle makes 5 spirals
        cmd.linear.x = 0.16 * elapsed
        cmd.angular.z = 1.0

        if 5.4 <= pose.x <= 5.6 and pose.y >= 11 - 0.5:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
    
        self.cmd_vel_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Spiral()
    rclpy.spin(node)
    rclpy.shutdown()