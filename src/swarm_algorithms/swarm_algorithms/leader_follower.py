#!/usr/bin/env python3
from termios import TIOCGWINSZ
from turtle import distance

import rclpy
from rclpy.node import Node 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class leader_follower(Node):
    def __init__(self):
        super().__init__('leader_follow')
        self.pose1=self.create_subscription(Odometry,'/TB3_1/odom',
                                            self.leader_odom_callback,5)
        self.pose2=self.create_subscription(Odometry,'/TB3_2/odom',
                                             self.follower_odom_callback,5)
        self.vel2=self.create_publisher(Twist,'/TB3_2/cmd_vel',
                                        5)
        self.timer = self.create_timer(0.05, self.control_loop)
    def leader_odom_callback(self,msg):
        self.leader_x=msg.pose.pose.position.x
        self.leader_y=msg.pose.pose.position.y

        self.get_logger().info(f'x={self.leader_x} , y={self.leader_y}')
        
    def follower_odom_callback(self,msg):
        self.follower_x=msg.pose.pose.position.x
        self.follower_y=msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        self.follower_theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.get_logger().info(f'x={self.follower_x} , y={self.follower_y}')

    def control_loop(self):
        dx=self.leader_x-self.follower_x
        dy=self.leader_y-self.follower_y
        target_angle = math.atan2(dy, dx)
        heading_error = target_angle - self.follower_theta
        heading_error = math.atan2(
            math.sin(heading_error),
            math.cos(heading_error)
        )
        distance=math.sqrt(dx**2 +dy**2)
        kp=1.5
        cmd = Twist()
        if abs(heading_error)>0.1:
            cmd.angular.z=kp*heading_error
            cmd.linear.x = 0.0

        elif distance>0.5:
            cmd.linear.x = 0.2
        self.vel2.publish(cmd)
        
def main(args=None):
    rclpy.init(args=args)
    node=leader_follower()
    rclpy.spin(node)
    rclpy.shutdown()