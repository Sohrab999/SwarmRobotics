#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class consensus(Node):
    def __init__(self):
        super().__init__('consensus')
        self.num_robots=4
        self.x=[0.0]*self.num_robots
        self.y=[0.0]*self.num_robots
        self.theta=[0.0]*self.num_robots
        self.pose_subscribers = []
        self.cmd_vel_publishers = []
        self.neighbours=[]
        for i in range(0,self.num_robots):
            left = (i - 1) % self.num_robots
            right = (i + 1) % self.num_robots

            self.neighbours.append([left, right])
        for i in range(0,self.num_robots):
            topic=f'/TB3_{i+1}/odom'
            sub=self.create_subscription(Odometry,topic,
                                         lambda msg,index=i:
                                         self.odom_callback(msg,index),5)
            self.pose_subscribers.append(sub)
        for i in range(0,self.num_robots):
            topic=f'/TB3_{i+1}/cmd_vel'
            pub=self.create_publisher(Twist,topic,5)
            self.cmd_vel_publishers.append(pub)

        self.timer = self.create_timer(0.05, self.control)
        

    def odom_callback(self, msg, index):

        self.x[index] = msg.pose.pose.position.x
        self.y[index] = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.theta[index] = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z)
        )
    def control(self):
        
        for  i  in range(0,self.num_robots):
            desired_x=(self.x[self.neighbours[i][0]]+self.x[self.neighbours[i][1]])/2.0
            dx=desired_x-self.x[i]
            desired_y=(self.y[self.neighbours[i][0]]+self.y[self.neighbours[i][1]])/2.0
            dy=desired_y-self.y[i]
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)
            
            heading_error = target_angle - self.theta[i]
            heading_error = math.atan2(
                        math.sin(heading_error),
                        math.cos(heading_error)
                    )
            cmd = Twist()
    
            kp_ang = 1.5
    
            if abs(heading_error) > 0.1:
                cmd.angular.z = kp_ang * heading_error
                cmd.linear.x = 0.0
    
            elif distance > 0.5:
                cmd.linear.x = 0.2
                cmd.angular.z = 0.0
    
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            self.cmd_vel_publishers[i].publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node=consensus()
    rclpy.spin(node)
    rclpy.shutdown()