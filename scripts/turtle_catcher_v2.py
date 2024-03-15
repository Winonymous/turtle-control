#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose

from geometry_msgs.msg import Twist

from math import atan, degrees, pi, sqrt

from turtle_control.msg import Turtle2Available

import time


class TurtleCatcher(Node):
    x_1 = x_2 = y_1 = y_2 = 0.0
    
    def __init__(self):
        super().__init__("goal_action_server")

        self.is_turtle2_available_pub_  = self.create_subscription(Turtle2Available, "/turtleavailable", self.is_turtle2_available, 10)
        self.t2_pose_sub_ = self.create_subscription(Pose, "/catch/pose", self.t2_pose_callback,  10)
        self.t1_pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.t1_pose_callback,  10)

        self.turtle2 = False

        self.speed_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.speed = Twist()
    
    def is_turtle2_available(self, output):
        self.turtle2 = output.isavailable
        self.finished_turning = False

    def t1_pose_callback(self, turtle_pose):
        self.x_1 = turtle_pose.x
        self.y_1 = turtle_pose.y
        self.catcherangle = turtle_pose.theta

        if (self.turtle2):
            self.move_goalward()

    def t2_pose_callback(self, turtle_pose):
        self.x_2 = turtle_pose.x
        self.y_2 = turtle_pose.y

    def move_goalward(self):
        angle = self.get_angle()

        if self.catcherangle < 0:
            self.catcherangle = (2 * pi) + self.catcherangle

        difference = round(angle - self.catcherangle)

        if (difference == 0.0):
            self.speed.angular.z = 0.0
        else:
            self.speed.angular.z = pi


        dist = self.get_distance() * 1.2

        self.speed.linear.x = dist
            
        self.speed_pub_.publish(self.speed)  

    def get_angle(self):

        self.adj = self.x_2 - self.x_1
        self.opp = self.y_2 - self.y_1

        if (self.x_2 > 0.0 and self.y_2 > 0.0):
            angle = atan(self.opp/self.adj)
            if (self.adj < 0):
                angle = pi + angle
            elif (self.opp < 0):
                angle = (2*pi) + angle

            return angle
        
    def get_distance(self):

        self.adj = self.x_2 - self.x_1
        self.opp = self.y_2 - self.y_1


        dist = sqrt((self.adj*self.adj) + (self.opp*self.opp)) 

        return dist            

def main(args = None):
    rclpy.init(args=args)
    turner = TurtleCatcher()
    turner.get_logger().info("Catcher Started")
    rclpy.spin(turner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()