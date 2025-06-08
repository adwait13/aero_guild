import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__('turtle_controller')

        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.state = 0
        
        self.get_logger().info('Controller started...')

    def pose_callback(self, pose: Pose):
        MAX = 11.08888
        cmd = Twist()

        #go straight hit right wall
        if self.state == 0:
            cmd.linear.x = 2.0
            if pose.x >= MAX - 0.1:
                self.state = 1

        #rotate
        elif self.state == 1:
            target_theta = math.pi / 4 * 3
            if abs(pose.theta - target_theta) > 0.01:
                cmd.angular.z = 1.0
            else:
                self.state = 2

        #go hit the upper wall
        elif self.state == 2:
            cmd.linear.x = 2.0
            if pose.y >= MAX - 0.01:
                self.state = 3

        #rotate
        elif self.state == 3:            
            target_theta = - math.pi / 4 * 3
            if abs(pose.theta - target_theta) > 0.01:
                cmd.angular.z = 1.0
            else:
                self.state = 4

        #go and hit left wall
        elif self.state == 4:
            cmd.linear.x = 2.0
            if pose.x <= 0.01:
                self.state = 5

        #rotate
        elif self.state == 5:            
            target_theta = -math.pi / 4
            if abs(pose.theta - target_theta) > 0.01:
                cmd.angular.z = 1.0
            else:
                self.state = 6

        #go and hit bottom wall
        elif self.state == 6:
            cmd.linear.x = 2.0
            if pose.y <= 0.01:
                self.state = 7

        #rotate
        elif self.state == 7:            
            target_theta = math.pi / 4
            if abs(pose.theta - target_theta) > 0.01:
                cmd.angular.z = 1.0
            else:
                self.state = 8

        #go and hit right wall
        elif self.state == 8:
            cmd.linear.x = 2.0
            if pose.x >= MAX - 0.01:
                self.state = 9

        #rotate
        elif self.state == 9:            
            target_theta = math.pi / 4 * 3
            if abs(pose.theta - target_theta) > 0.01:
                cmd.angular.z = 1.0
            else:
                self.state = 2

        self.cmd_vel_publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()