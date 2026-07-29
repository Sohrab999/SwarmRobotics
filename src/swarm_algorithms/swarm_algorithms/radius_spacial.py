#!/usr/bin/env python3 

from operator import ne

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class radius_spacial(Node):
    def __init__(self):
        super().__init__('radius_spacial')
        self.num_robots=4
        self.x=[0.0]*self.num_robots
        self.y=[0.0]*self.num_robots
        self.theta=[0.0]*self.num_robots
        self.pose_subscribers = []
        self.cmd_vel_publishers = []
        
        self.r=3.0
        self.grid_size = self.r
        self.safe_distance=1.0
        self.k_repulsion=1.0
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

    def get_neighbours(self,i,grid):
        neighbours=[]
        cell_x = int(self.x[i] // self.grid_size)
        cell_y = int(self.y[i] // self.grid_size)
        for dx_cell in [-1,0,1]:
            for dy_cell in [-1,0,1]:
                neighbour_cell=(cell_x+dx_cell
                                ,dy_cell+cell_y)

                if neighbour_cell not in grid:
                    continue

                for j in grid[neighbour_cell]:
                    if i==j:
                        continue
                    dx=self.x[j]-self.x[i]
                    dy=self.y[j]-self.y[i]

                    d=math.sqrt(dx**2+dy**2)

                    if d < self.r:
                        neighbours.append(j)
        return neighbours
            
    
    def control(self):
        grid = {}

        rep_x = 0.0
        rep_y = 0.0

        
        for i in range(self.num_robots):

            cell_x = int(self.x[i] // self.grid_size)
            cell_y = int(self.y[i] // self.grid_size)

            cell = (cell_x, cell_y)

            if cell not in grid:
                grid[cell] = []

            grid[cell].append(i)
        
        for i in range(0,self.num_robots):
            cmd=Twist()
            neighbours = self.get_neighbours(i,grid)
            for j in neighbours:
            
                dx = self.x[i] - self.x[j]
                dy = self.y[i] - self.y[j]
    
                d = math.sqrt(dx*dx + dy*dy)
    
                if d < self.safe_distance and d > 0.001:
    
                    strength = self.k_repulsion * (self.safe_distance - d) / self.safe_distance
    
                    rep_x += strength * dx / d
                    rep_y += strength * dy / d
            
            if len(neighbours)==0:
                cmd.linear.x = 0.1
                cmd.angular.z = 0.4


                self.cmd_vel_publishers[i].publish(cmd)

                continue

            avg_x=0.0
            avg_y=0.0

            for j in neighbours:
                avg_x+=self.x[j]
                avg_y+=self.y[j]

            avg_x /= len(neighbours)

            avg_y /= len(neighbours)

            att_x = avg_x - self.x[i]
            att_y = avg_y - self.y[i]

            dx = att_x + rep_x
            dy = att_y + rep_y


            distance = math.sqrt(dx**2 + dy**2)

            desired_angle = math.atan2(dy,dx)
            angle_error = (desired_angle-self.theta[i])
            angle_error = math.atan2(math.sin(angle_error),math.cos(angle_error))
            k_linear = 0.3
            k_angular = 1.5
            if abs(angle_error) > 0.15:
                cmd.angular.z = (k_angular*angle_error)
                cmd.linear.x = 0.0

            else:

                cmd.linear.x = (k_linear*distance)
                cmd.angular.z = (k_angular*angle_error)

                if distance < 0.75:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
            self.cmd_vel_publishers[i].publish(cmd)



def main(args=None):
    rclpy.init(args=args)
    node=radius_spacial()
    rclpy.spin(node)
    rclpy.shutdown()