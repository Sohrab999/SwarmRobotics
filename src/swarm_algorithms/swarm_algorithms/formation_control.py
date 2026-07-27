#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class FormationControl(Node):

    def __init__(self):
        super().__init__('formation_control')

        self.leader_x = 0.0
        self.leader_y = 0.0

        self.follower_x = 0.0
        self.follower_y = 0.0
        self.follower_theta = 0.0

        self.pose1 = self.create_subscription(
            Odometry,
            '/TB3_1/odom',
            self.leader_odom_callback,
            5
        )

        self.pose2 = self.create_subscription(
            Odometry,
            '/TB3_2/odom',
            self.follower_odom_callback,
            5
        )

        self.vel2 = self.create_publisher(
            Twist,
            '/TB3_2/cmd_vel',
            5
        )

        self.timer = self.create_timer(0.05, self.control_loop)

    def leader_odom_callback(self, msg):

        self.leader_x = msg.pose.pose.position.x
        self.leader_y = msg.pose.pose.position.y

    def follower_odom_callback(self, msg):

        self.follower_x = msg.pose.pose.position.x
        self.follower_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.follower_theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def control_loop(self):

        offset_x = -1.0
        offset_y = 0.0

        desired_x = self.leader_x + offset_x
        desired_y = self.leader_y + offset_y

        dx = desired_x - self.follower_x
        dy = desired_y - self.follower_y

        distance = math.sqrt(dx**2 + dy**2)

        target_angle = math.atan2(dy, dx)

        heading_error = target_angle - self.follower_theta

        heading_error = math.atan2(
            math.sin(heading_error),
            math.cos(heading_error)
        )

        cmd = Twist()

        kp_ang = 1.5

        if abs(heading_error) > 0.1:
            cmd.angular.z = kp_ang * heading_error
            cmd.linear.x = 0.0

        elif distance > 0.05:
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.vel2.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = FormationControl()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()